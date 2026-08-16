#include "fr3_control_sim/robot_model.hpp"

#include <Eigen/Core>

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

void test_cad_urdf() {
  fr3_control_sim::RobotModel model(FR3_SIM_CAD_URDF);
  if (model.nq() != 7 || model.nv() != 7) {
    throw std::runtime_error("CAD URDF model is not 7-DoF");
  }
  if (model.end_effector_frame() != "link_7") {
    throw std::runtime_error("CAD URDF did not auto-select link_7");
  }

  const auto names = model.joint_names();
  if (names.size() != 7 || names.front() != "joint_1" ||
      names.back() != "joint_7") {
    throw std::runtime_error("CAD URDF joint names were not loaded");
  }

  const Eigen::VectorXd home = model.home_configuration();
  Eigen::VectorXd expected(7);
  expected << 0.0, -M_PI / 4.0, 0.0, 3.0 * M_PI / 4.0, 0.0,
      -M_PI / 2.0, -M_PI / 4.0;
  if (!home.isApprox(expected, 1e-12)) {
    throw std::runtime_error("CAD URDF ready configuration is incorrect");
  }

  Eigen::VectorXd target_q = home;
  target_q[0] += 0.08;
  target_q[1] += 0.06;
  target_q[2] -= 0.07;
  target_q[3] -= 0.05;
  target_q[4] += 0.05;
  target_q[5] += 0.04;
  target_q[6] -= 0.06;
  const Eigen::Matrix4d target = model.forward_kinematics(target_q);

  fr3_control_sim::IKOptions options;
  options.max_iterations = 1500;
  options.max_retries = 4;
  options.tolerance = 1e-6;
  const auto result = model.inverse_kinematics(target, home, options);
  if (!result.success) {
    throw std::runtime_error("CAD URDF IK did not converge, error=" +
                             std::to_string(result.error));
  }
  const Eigen::Matrix4d recovered = model.forward_kinematics(result.q);
  if ((recovered.topRightCorner<3, 1>() - target.topRightCorner<3, 1>())
          .norm() > 1e-5) {
    throw std::runtime_error("CAD URDF IK/FK round trip is inaccurate");
  }
}

} // namespace

int main() {
  try {
    fr3_control_sim::RobotModel model(FR3_SIM_DEFAULT_URDF);
    if (model.nq() != 7 || model.nv() != 7) {
      throw std::runtime_error("Reduced FR3 model is not 7-DoF");
    }

    const Eigen::VectorXd q_home = model.home_configuration();
    const Eigen::Matrix4d home_pose = model.forward_kinematics(q_home);
    if (!home_pose.array().isFinite().all()) {
      throw std::runtime_error("FK returned non-finite values");
    }

    Eigen::VectorXd q_target = q_home;
    q_target[0] += 0.25;
    q_target[1] += 0.15;
    q_target[2] -= 0.20;
    q_target[4] += 0.20;
    q_target[6] -= 0.25;
    const Eigen::Matrix4d target_pose = model.forward_kinematics(q_target);

    fr3_control_sim::IKOptions options;
    options.max_iterations = 1500;
    options.max_retries = 4;
    options.tolerance = 1e-6;
    const auto result = model.inverse_kinematics(target_pose, q_home, options);
    if (!result.success) {
      throw std::runtime_error("IK did not converge, error=" +
                               std::to_string(result.error));
    }

    const Eigen::Matrix4d recovered = model.forward_kinematics(result.q);
    const double position_error =
        (recovered.topRightCorner<3, 1>() - target_pose.topRightCorner<3, 1>())
            .norm();
    if (position_error > 1e-5) {
      throw std::runtime_error("IK/FK position round-trip error is too large");
    }

    // FR3 is redundant: the same TCP pose can be reached with different elbow
    // postures.  The null-space objective should recover the ready/home
    // posture when solving the home target from a deliberately skewed seed.
    Eigen::VectorXd q_skewed(7);
    q_skewed << -27.75, -48.05, 17.44, -134.54, 12.89, 87.89, 29.59;
    q_skewed *= M_PI / 180.0;
    const Eigen::Matrix4d near_home_target =
        fr3_control_sim::pose_from_xyz_rpy(
            Eigen::Vector3d(0.3069, 0.0, 0.4869),
            Eigen::Vector3d(-3.1416, 0.0, 0.0));
    fr3_control_sim::IKOptions posture_options;
    posture_options.max_iterations = 1000;
    posture_options.max_retries = 0;
    posture_options.posture_gain = 0.1;
    const auto posture_result =
        model.inverse_kinematics(near_home_target, q_skewed, posture_options);
    if (!posture_result.success) {
      throw std::runtime_error("Null-space posture IK did not converge");
    }
    const double posture_distance = (posture_result.q - q_home).norm();
    if (posture_distance > 1e-3) {
      throw std::runtime_error(
          "Null-space posture objective did not recover home");
    }

    const Eigen::MatrixXd trajectory =
        model.minimum_jerk_trajectory(q_home, result.q, 1.0, 0.02);
    if (trajectory.rows() != 51 || trajectory.cols() != 7 ||
        !trajectory.row(0).isApprox(q_home.transpose()) ||
        !trajectory.row(trajectory.rows() - 1).isApprox(result.q.transpose())) {
      throw std::runtime_error("Minimum-jerk trajectory endpoints are invalid");
    }

    options.max_retries = 8;
    for (unsigned int seed = 1000; seed < 1020; ++seed) {
      const Eigen::Matrix4d reachable_pose =
          model.forward_kinematics(model.random_configuration(seed));
      const auto random_result =
          model.inverse_kinematics(reachable_pose, q_home, options);
      if (!random_result.success) {
        throw std::runtime_error(
            "IK random reachable-pose regression failed at seed " +
            std::to_string(seed));
      }
    }

    test_cad_urdf();

    std::cout << "FR3 C++ smoke test passed\n"
              << "  joints: " << model.nq() << "\n"
              << "  home tcp xyz: "
              << home_pose.topRightCorner<3, 1>().transpose() << "\n"
              << "  IK error: " << result.error << "\n"
              << "  random reachable IK: 20/20\n";
    std::cout << "  CAD URDF load/FK/IK: passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "FR3 C++ smoke test failed: " << error.what() << '\n';
    return 1;
  }
}
