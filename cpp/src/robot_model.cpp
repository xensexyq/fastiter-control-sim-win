#include "fr3_control_sim/robot_model.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/math/rpy.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>

namespace fr3_control_sim {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

bool is_finite_matrix(const Eigen::Matrix4d &matrix) {
  return matrix.array().isFinite().all();
}

bool has_joint(const pinocchio::Model &model, const std::string &name) {
  return model.existJointName(name);
}

std::vector<pinocchio::JointIndex>
finger_joint_ids(const pinocchio::Model &model) {
  std::vector<pinocchio::JointIndex> ids;
  for (const char *name : {"fr3_finger_joint1", "fr3_finger_joint2"}) {
    if (has_joint(model, name)) {
      ids.push_back(model.getJointId(name));
    }
  }
  return ids;
}

} // namespace

RobotModel::RobotModel(const std::string &urdf_path,
                       const std::string &end_effector_frame,
                       double finger_position)
    : urdf_path_(
          std::filesystem::absolute(urdf_path).lexically_normal().string()),
      end_effector_frame_(end_effector_frame),
      finger_position_(finger_position), data_(model_) {
  if (!std::filesystem::is_regular_file(urdf_path_)) {
    throw std::invalid_argument("URDF file does not exist: " + urdf_path_);
  }
  if (!std::isfinite(finger_position_) || finger_position_ < 0.0 ||
      finger_position_ > 0.04) {
    throw std::invalid_argument(
        "finger_position must be in [0.0, 0.04] meters");
  }

  pinocchio::Model full_model;
  pinocchio::urdf::buildModel(urdf_path_, full_model);

  const auto locked_joints = finger_joint_ids(full_model);
  if (locked_joints.size() != 2U) {
    throw std::runtime_error(
        "Expected official FR3 hand joints fr3_finger_joint1 and "
        "fr3_finger_joint2 in URDF");
  }

  Eigen::VectorXd reference = pinocchio::neutral(full_model);
  for (const auto joint_id : locked_joints) {
    const int idx_q = full_model.joints[joint_id].idx_q();
    if (idx_q < 0 || full_model.joints[joint_id].nq() != 1) {
      throw std::runtime_error(
          "FR3 finger joint has an unsupported configuration");
    }
    reference[idx_q] = finger_position_;
  }

  model_ = pinocchio::buildReducedModel(full_model, locked_joints, reference);
  data_ = pinocchio::Data(model_);

  if (model_.nq != 7 || model_.nv != 7) {
    std::ostringstream message;
    message << "Expected a 7-DoF FR3 arm after locking the hand, got nq="
            << model_.nq << " nv=" << model_.nv;
    throw std::runtime_error(message.str());
  }
  (void)resolve_frame(end_effector_frame_);
}

std::vector<std::string> RobotModel::joint_names() const {
  std::vector<std::string> names;
  names.reserve(static_cast<std::size_t>(model_.nq));
  for (pinocchio::JointIndex joint_id = 1; joint_id < model_.joints.size();
       ++joint_id) {
    if (model_.joints[joint_id].nq() > 0) {
      names.push_back(model_.names[joint_id]);
    }
  }
  return names;
}

std::vector<std::pair<double, double>> RobotModel::joint_limits() const {
  std::vector<std::pair<double, double>> limits;
  limits.reserve(static_cast<std::size_t>(model_.nq));
  for (int index = 0; index < model_.nq; ++index) {
    limits.emplace_back(model_.lowerPositionLimit[index],
                        model_.upperPositionLimit[index]);
  }
  return limits;
}

std::vector<std::string> RobotModel::frame_names() const {
  std::vector<std::string> names;
  names.reserve(model_.frames.size());
  for (const auto &frame : model_.frames) {
    names.push_back(frame.name);
  }
  return names;
}

Eigen::VectorXd RobotModel::home_configuration() const {
  Eigen::VectorXd q(7);
  q << 0.0, -kPi / 4.0, 0.0, -3.0 * kPi / 4.0, 0.0, kPi / 2.0, kPi / 4.0;
  return clamp_configuration(q);
}

Eigen::VectorXd RobotModel::random_configuration(unsigned int seed) const {
  std::mt19937 generator(seed);
  Eigen::VectorXd q(model_.nq);
  for (int index = 0; index < model_.nq; ++index) {
    double lower = model_.lowerPositionLimit[index];
    double upper = model_.upperPositionLimit[index];
    if (!std::isfinite(lower)) {
      lower = -kPi;
    }
    if (!std::isfinite(upper)) {
      upper = kPi;
    }
    std::uniform_real_distribution<double> distribution(lower, upper);
    q[index] = distribution(generator);
  }
  return q;
}

void RobotModel::validate_configuration(const Eigen::VectorXd &q) const {
  if (q.size() != model_.nq) {
    std::ostringstream message;
    message << "Expected q with " << model_.nq << " values, got " << q.size();
    throw std::invalid_argument(message.str());
  }
  if (!q.array().isFinite().all()) {
    throw std::invalid_argument("q contains NaN or infinity");
  }
}

pinocchio::FrameIndex
RobotModel::resolve_frame(const std::string &frame_name) const {
  const std::string &requested =
      frame_name.empty() ? end_effector_frame_ : frame_name;
  for (pinocchio::FrameIndex index = 0; index < model_.frames.size(); ++index) {
    if (model_.frames[index].name == requested) {
      return index;
    }
  }
  throw std::invalid_argument("Unknown frame: " + requested);
}

Eigen::VectorXd
RobotModel::clamp_configuration(const Eigen::VectorXd &q) const {
  validate_configuration(q);
  Eigen::VectorXd clamped = q;
  for (int index = 0; index < model_.nq; ++index) {
    const double lower = model_.lowerPositionLimit[index];
    const double upper = model_.upperPositionLimit[index];
    if (std::isfinite(lower)) {
      clamped[index] = std::max(clamped[index], lower);
    }
    if (std::isfinite(upper)) {
      clamped[index] = std::min(clamped[index], upper);
    }
  }
  return clamped;
}

Eigen::Matrix4d
RobotModel::forward_kinematics(const Eigen::VectorXd &q,
                               const std::string &frame_name) const {
  validate_configuration(q);
  const auto frame_id = resolve_frame(frame_name);
  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
  return data_.oMf[frame_id].toHomogeneousMatrix();
}

Eigen::MatrixXd RobotModel::jacobian(const Eigen::VectorXd &q,
                                     const std::string &frame_name) const {
  validate_configuration(q);
  const auto frame_id = resolve_frame(frame_name);
  pinocchio::computeJointJacobians(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
  pinocchio::Data::Matrix6x result(6, model_.nv);
  result.setZero();
  pinocchio::getFrameJacobian(model_, data_, frame_id, pinocchio::LOCAL,
                              result);
  return result;
}

std::map<std::string, Eigen::Matrix4d>
RobotModel::frame_placements(const Eigen::VectorXd &q) const {
  validate_configuration(q);
  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);

  std::map<std::string, Eigen::Matrix4d> placements;
  for (pinocchio::FrameIndex index = 0; index < model_.frames.size(); ++index) {
    if (model_.frames[index].type == pinocchio::BODY) {
      placements[model_.frames[index].name] =
          data_.oMf[index].toHomogeneousMatrix();
    }
  }
  return placements;
}

RobotModel::ErrorState
RobotModel::pose_error(const Eigen::VectorXd &q, const pinocchio::SE3 &target,
                       pinocchio::FrameIndex frame_id) const {
  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
  const pinocchio::SE3 current_to_target =
      data_.oMf[frame_id].inverse() * target;

  ErrorState state;
  state.vector = pinocchio::log6(current_to_target).toVector();
  state.position_norm = state.vector.head<3>().norm();
  state.orientation_norm = state.vector.tail<3>().norm();
  state.norm = state.vector.norm();
  return state;
}

IKResult RobotModel::inverse_kinematics_once(const pinocchio::SE3 &target,
                                             const Eigen::VectorXd &q_seed,
                                             pinocchio::FrameIndex frame_id,
                                             const IKOptions &options) const {
  Eigen::VectorXd q = clamp_configuration(q_seed);
  ErrorState state = pose_error(q, target, frame_id);
  int stalled_iterations = 0;

  for (int iteration = 0; iteration < options.max_iterations; ++iteration) {
    if (state.norm <= options.tolerance) {
      return IKResult{q,
                      true,
                      iteration,
                      1,
                      state.norm,
                      state.position_norm,
                      state.orientation_norm};
    }

    pinocchio::computeJointJacobians(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_);
    pinocchio::Data::Matrix6x jacobian_matrix(6, model_.nv);
    jacobian_matrix.setZero();
    pinocchio::getFrameJacobian(model_, data_, frame_id, pinocchio::LOCAL,
                                jacobian_matrix);

    // Differentiate log6(current^-1 * target) exactly, following the
    // Pinocchio closed-loop IK formulation.  This is noticeably more robust
    // than using the raw LOCAL Jacobian for targets far from the seed.
    const pinocchio::SE3 current_to_target =
        data_.oMf[frame_id].inverse() * target;
    const Eigen::MatrixXd task_jacobian =
        -pinocchio::Jlog6(current_to_target.inverse()) * jacobian_matrix;

    Eigen::Matrix<double, 6, 6> normal =
        task_jacobian * task_jacobian.transpose();
    const double adaptive_damping =
        options.damping * std::max(1.0, 10.0 * state.norm);
    normal.diagonal().array() += adaptive_damping;
    Eigen::VectorXd delta = -options.step_size * task_jacobian.transpose() *
                            normal.ldlt().solve(state.vector);

    const double delta_norm = delta.norm();
    if (options.max_step_norm > 0.0 && delta_norm > options.max_step_norm) {
      delta *= options.max_step_norm / delta_norm;
    }

    bool improved = false;
    double alpha = 1.0;
    for (int search = 0; search < options.line_search_steps; ++search) {
      Eigen::VectorXd candidate =
          pinocchio::integrate(model_, q, alpha * delta);
      candidate = clamp_configuration(candidate);
      const ErrorState candidate_state =
          pose_error(candidate, target, frame_id);
      if (candidate_state.norm + 1e-12 < state.norm) {
        q = candidate;
        state = candidate_state;
        improved = true;
        break;
      }
      alpha *= 0.5;
    }

    if (improved) {
      stalled_iterations = 0;
    } else {
      ++stalled_iterations;
      if (stalled_iterations >= 10) {
        return IKResult{q,
                        false,
                        iteration + 1,
                        1,
                        state.norm,
                        state.position_norm,
                        state.orientation_norm};
      }
    }
  }

  return IKResult{q,
                  false,
                  options.max_iterations,
                  1,
                  state.norm,
                  state.position_norm,
                  state.orientation_norm};
}

IKResult RobotModel::inverse_kinematics(const Eigen::Matrix4d &target_matrix,
                                        const Eigen::VectorXd &q_seed,
                                        const IKOptions &options) const {
  validate_configuration(q_seed);
  if (!is_finite_matrix(target_matrix)) {
    throw std::invalid_argument("target contains NaN or infinity");
  }
  const Eigen::RowVector4d homogeneous_row(0.0, 0.0, 0.0, 1.0);
  if (!target_matrix.row(3).isApprox(homogeneous_row, 1e-8)) {
    throw std::invalid_argument("target last row must be [0, 0, 0, 1]");
  }
  if (options.max_iterations <= 0 || options.max_retries < 0 ||
      options.tolerance <= 0.0 || options.damping < 0.0 ||
      options.step_size <= 0.0 || options.max_step_norm < 0.0 ||
      options.line_search_steps <= 0) {
    throw std::invalid_argument("Invalid IK options");
  }

  Eigen::Matrix3d rotation = target_matrix.topLeftCorner<3, 3>();
  const double orthogonality_error =
      (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm();
  if (orthogonality_error > 1e-5 || rotation.determinant() <= 0.0) {
    throw std::invalid_argument(
        "target rotation must be a proper rotation matrix");
  }
  Eigen::Quaterniond quaternion(rotation);
  if (quaternion.norm() < std::numeric_limits<double>::epsilon()) {
    throw std::invalid_argument("target rotation is singular");
  }
  quaternion.normalize();
  const pinocchio::SE3 target(quaternion.toRotationMatrix(),
                              target_matrix.topRightCorner<3, 1>());
  const auto frame_id = resolve_frame(end_effector_frame_);

  IKResult best = inverse_kinematics_once(target, q_seed, frame_id, options);
  best.attempts = 1;
  if (best.success) {
    return best;
  }

  for (int retry = 0; retry < options.max_retries; ++retry) {
    const Eigen::VectorXd retry_seed = random_configuration(
        options.random_seed + static_cast<unsigned int>(retry));
    IKResult candidate =
        inverse_kinematics_once(target, retry_seed, frame_id, options);
    candidate.attempts = retry + 2;
    if (candidate.error < best.error) {
      best = candidate;
    }
    if (candidate.success) {
      return candidate;
    }
  }
  best.attempts = options.max_retries + 1;
  return best;
}

Eigen::MatrixXd
RobotModel::minimum_jerk_trajectory(const Eigen::VectorXd &q_start,
                                    const Eigen::VectorXd &q_goal,
                                    double duration, double dt) const {
  validate_configuration(q_start);
  validate_configuration(q_goal);
  if (!std::isfinite(duration) || !std::isfinite(dt) || duration <= 0.0 ||
      dt <= 0.0) {
    throw std::invalid_argument("duration and dt must be finite and positive");
  }

  const int intervals = std::max(1, static_cast<int>(std::ceil(duration / dt)));
  Eigen::MatrixXd trajectory(intervals + 1, model_.nq);
  for (int index = 0; index <= intervals; ++index) {
    const double s = static_cast<double>(index) / intervals;
    const double s2 = s * s;
    const double s3 = s2 * s;
    const double blend = 10.0 * s3 - 15.0 * s3 * s + 6.0 * s3 * s2;
    trajectory.row(index) = (q_start + blend * (q_goal - q_start)).transpose();
  }
  return trajectory;
}

Eigen::Matrix4d pose_from_xyz_rpy(const Eigen::Vector3d &xyz,
                                  const Eigen::Vector3d &rpy) {
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  pose.topLeftCorner<3, 3>() = pinocchio::rpy::rpyToMatrix(rpy);
  pose.topRightCorner<3, 1>() = xyz;
  return pose;
}

Eigen::Vector3d rpy_from_pose(const Eigen::Matrix4d &pose) {
  if (!is_finite_matrix(pose)) {
    throw std::invalid_argument("pose contains NaN or infinity");
  }
  return pinocchio::rpy::matrixToRpy(pose.topLeftCorner<3, 3>());
}

} // namespace fr3_control_sim
