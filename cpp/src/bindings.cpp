#include "fr3_control_sim/robot_model.hpp"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using fr3_control_sim::IKOptions;
using fr3_control_sim::IKResult;
using fr3_control_sim::RobotModel;

PYBIND11_MODULE(_fr3_sim, module) {
  module.doc() =
      "FR3 forward/inverse kinematics implemented in C++ with Pinocchio";

  py::class_<IKOptions>(module, "IKOptions")
      .def(py::init<>())
      .def_readwrite("max_iterations", &IKOptions::max_iterations)
      .def_readwrite("max_retries", &IKOptions::max_retries)
      .def_readwrite("tolerance", &IKOptions::tolerance)
      .def_readwrite("damping", &IKOptions::damping)
      .def_readwrite("step_size", &IKOptions::step_size)
      .def_readwrite("max_step_norm", &IKOptions::max_step_norm)
      .def_readwrite("posture_gain", &IKOptions::posture_gain)
      .def_readwrite("line_search_steps", &IKOptions::line_search_steps)
      .def_readwrite("random_seed", &IKOptions::random_seed)
      .def("__repr__", [](const IKOptions &options) {
        return "IKOptions(max_iterations=" +
               std::to_string(options.max_iterations) +
               ", max_retries=" + std::to_string(options.max_retries) +
               ", tolerance=" + std::to_string(options.tolerance) +
               ", posture_gain=" + std::to_string(options.posture_gain) +
               ")";
      });

  py::class_<IKResult>(module, "IKResult")
      .def_readonly("q", &IKResult::q)
      .def_readonly("success", &IKResult::success)
      .def_readonly("iterations", &IKResult::iterations)
      .def_readonly("attempts", &IKResult::attempts)
      .def_readonly("error", &IKResult::error)
      .def_readonly("position_error", &IKResult::position_error)
      .def_readonly("orientation_error", &IKResult::orientation_error)
      .def("__repr__", [](const IKResult &result) {
        return "IKResult(success=" +
               std::string(result.success ? "True" : "False") +
               ", error=" + std::to_string(result.error) +
               ", iterations=" + std::to_string(result.iterations) +
               ", attempts=" + std::to_string(result.attempts) + ")";
      });

  py::class_<RobotModel>(module, "RobotModel")
      .def(py::init<const std::string &, const std::string &, double>(),
           py::arg("urdf_path"), py::arg("end_effector_frame") = "",
           py::arg("finger_position") = 0.02)
      .def_property_readonly("nq", &RobotModel::nq)
      .def_property_readonly("nv", &RobotModel::nv)
      .def_property_readonly("urdf_path", &RobotModel::urdf_path)
      .def_property_readonly("end_effector_frame",
                             &RobotModel::end_effector_frame)
      .def_property_readonly("finger_position", &RobotModel::finger_position)
      .def_property_readonly("joint_names", &RobotModel::joint_names)
      .def_property_readonly("joint_limits", &RobotModel::joint_limits)
      .def_property_readonly("frame_names", &RobotModel::frame_names)
      .def("home_configuration", &RobotModel::home_configuration)
      .def("random_configuration", &RobotModel::random_configuration,
           py::arg("seed") = 42)
      .def("forward_kinematics", &RobotModel::forward_kinematics, py::arg("q"),
           py::arg("frame_name") = "")
      .def("jacobian", &RobotModel::jacobian, py::arg("q"),
           py::arg("frame_name") = "")
      .def("frame_placements", &RobotModel::frame_placements, py::arg("q"))
      .def("inverse_kinematics", &RobotModel::inverse_kinematics,
           py::arg("target"), py::arg("q_seed"),
           py::arg("options") = IKOptions())
      .def("minimum_jerk_trajectory", &RobotModel::minimum_jerk_trajectory,
           py::arg("q_start"), py::arg("q_goal"), py::arg("duration"),
           py::arg("dt") = 0.02);

  module.def("pose_from_xyz_rpy", &fr3_control_sim::pose_from_xyz_rpy,
             py::arg("xyz"), py::arg("rpy"));
  module.def("rpy_from_pose", &fr3_control_sim::rpy_from_pose, py::arg("pose"));
}
