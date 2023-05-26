#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "franky.hpp"

namespace py = pybind11;
using namespace pybind11::literals; // to bring in the '_a' literal
using namespace franky;

template<int dims>
std::string vec_to_str(const Vector<dims> &vec) {
  std::stringstream ss;
  ss << "[";
  for (size_t i = 0; i < dims; i++) {
    ss << vec[i];
    if (i != dims - 1)
      ss << " ";
  }
  ss << "]";
  return ss.str();
}

std::string affine_to_str(const Affine &affine) {
  std::stringstream ss;
  ss << "(t=" << vec_to_str(affine.translation().eval())
     << ", q=" << vec_to_str(Eigen::Quaterniond(affine.rotation()).coeffs()) << ")";
  return ss.str();
}

PYBIND11_MODULE(_franky, m) {
  m.doc() = "High-Level Motion Library for the Franka Panda Robot";

  py::class_<Condition>(m, "Condition")
      .def("__repr__", &Condition::repr);

  py::class_<ExponentialImpedanceMotion>(m, "ExponentialImpedanceMotion")
      .def(py::init<>([](
          const Affine &target,
          ImpedanceMotion::TargetType target_type = ImpedanceMotion::TargetType::Absolute,
          double translational_stiffness = 2000,
          double rotational_stiffness = 200,
          std::optional<std::array<std::optional<double>, 6>> force_constraints = std::nullopt,
          double exponential_decay = 0.005) {
        Eigen::Vector<bool, 6> force_constraints_active = Eigen::Vector<bool, 6>::Zero();
        Eigen::Vector<double, 6> force_constraints_value;
        if (force_constraints.has_value()) {
          for (int i = 0; i < 6; i++) {
            force_constraints_value[i] = force_constraints.value()[i].value_or(NAN);
            force_constraints_active[i] = force_constraints.value()[i].has_value();
          }
        }
        return new ExponentialImpedanceMotion(
            target,
            {target_type, translational_stiffness, rotational_stiffness, force_constraints_value,
             force_constraints_active, exponential_decay});
      }));

  py::class_<ImpedanceMotion>(m, "ImpedanceMotion");

  py::class_<JointMotion>(m, "JointMotion")
      .def(py::init<>([](
          const Vector7d &target,
          double velocity_rel = 1.0,
          double acceleration_rel = 1.0,
          double jerk_rel = 1.0,
          bool return_when_finished = true) {
        return new JointMotion(target, {velocity_rel, acceleration_rel, jerk_rel, return_when_finished});
      }));

  py::class_<LinearImpedanceMotion>(m, "LinearImpedanceMotion")
      .def(py::init<>([](
          const Affine &target,
          double duration,
          ImpedanceMotion::TargetType target_type = ImpedanceMotion::TargetType::Absolute,
          double translational_stiffness = 2000,
          double rotational_stiffness = 200,
          std::optional<std::array<std::optional<double>, 6>> force_constraints = std::nullopt,
          bool return_when_finished = true,
          double finish_wait_factor = 1.2) {
        Eigen::Vector<bool, 6> force_constraints_active = Eigen::Vector<bool, 6>::Zero();
        Eigen::Vector<double, 6> force_constraints_value;
        if (force_constraints.has_value()) {
          for (int i = 0; i < 6; i++) {
            force_constraints_value[i] = force_constraints.value()[i].value_or(NAN);
            force_constraints_active[i] = force_constraints.value()[i].has_value();
          }
        }
        return new LinearImpedanceMotion(
            target, duration,
            {target_type, translational_stiffness, rotational_stiffness, force_constraints_value,
             force_constraints_active, return_when_finished, finish_wait_factor});
      }));

  py::class_<LinearMotion>(m, "LinearMotion")
      .def(py::init<>([](
          const RobotPose &target,
          bool relative = false,
          double velocity_rel = 1.0) {
        return new LinearMotion(target, relative, velocity_rel);
      }));

  py::class_<Measure>(m, "Measure")
      .def_property_readonly_static("FORCE_X", [](py::object) { return Measure::ForceX(); })
      .def_property_readonly_static("FORCE_Y", [](py::object) { return Measure::ForceY(); })
      .def_property_readonly_static("FORCE_Z", [](py::object) { return Measure::ForceZ(); })
      .def_property_readonly_static("TIME", [](py::object) { return Measure::Time(); })
      .def("__eq__", pybind11::overload_cast<const Measure &, const Measure &>(&operator==), py::is_operator())
      .def("__eq__", pybind11::overload_cast<const Measure &, double>(&operator==), py::is_operator())
      .def("__eq__", pybind11::overload_cast<double, const Measure &>(&operator==), py::is_operator())
      .def("__ne__", pybind11::overload_cast<const Measure &, const Measure &>(&operator!=), py::is_operator())
      .def("__ne__", pybind11::overload_cast<const Measure &, double>(&operator!=), py::is_operator())
      .def("__ne__", pybind11::overload_cast<double, const Measure &>(&operator!=), py::is_operator())
      .def("__gt__", pybind11::overload_cast<const Measure &, const Measure &>(&operator>), py::is_operator())
      .def("__gt__", pybind11::overload_cast<const Measure &, double>(&operator>), py::is_operator())
      .def("__gt__", pybind11::overload_cast<double, const Measure &>(&operator>), py::is_operator())
      .def("__ge__", pybind11::overload_cast<const Measure &, const Measure &>(&operator>=), py::is_operator())
      .def("__ge__", pybind11::overload_cast<const Measure &, double>(&operator>=), py::is_operator())
      .def("__ge__", pybind11::overload_cast<double, const Measure &>(&operator>=), py::is_operator())
      .def("__lt__", pybind11::overload_cast<const Measure &, const Measure &>(&operator<), py::is_operator())
      .def("__lt__", pybind11::overload_cast<const Measure &, double>(&operator<), py::is_operator())
      .def("__lt__", pybind11::overload_cast<double, const Measure &>(&operator<), py::is_operator())
      .def("__le__", pybind11::overload_cast<const Measure &, const Measure &>(&operator<=), py::is_operator())
      .def("__le__", pybind11::overload_cast<const Measure &, double>(&operator<=), py::is_operator())
      .def("__le__", pybind11::overload_cast<double, const Measure &>(&operator<=), py::is_operator())
      .def("__add__", pybind11::overload_cast<const Measure &, const Measure &>(&operator+), py::is_operator())
      .def("__add__", pybind11::overload_cast<const Measure &, double>(&operator+), py::is_operator())
      .def("__add__", pybind11::overload_cast<double, const Measure &>(&operator+), py::is_operator())
      .def("__sub__", pybind11::overload_cast<const Measure &, const Measure &>(&operator-), py::is_operator())
      .def("__sub__", pybind11::overload_cast<const Measure &, double>(&operator-), py::is_operator())
      .def("__sub__", pybind11::overload_cast<double, const Measure &>(&operator-), py::is_operator())
      .def("__mul__", pybind11::overload_cast<const Measure &, const Measure &>(&operator*), py::is_operator())
      .def("__mul__", pybind11::overload_cast<const Measure &, double>(&operator*), py::is_operator())
      .def("__mul__", pybind11::overload_cast<double, const Measure &>(&operator*), py::is_operator())
      .def("__div__", pybind11::overload_cast<const Measure &, const Measure &>(&operator/), py::is_operator())
      .def("__div__", pybind11::overload_cast<const Measure &, double>(&operator/), py::is_operator())
      .def("__div__", pybind11::overload_cast<double, const Measure &>(&operator/), py::is_operator())
      .def("__pow__", pybind11::overload_cast<const Measure &, const Measure &>(&measure_pow), py::is_operator())
      .def("__pow__", pybind11::overload_cast<const Measure &, double>(&measure_pow), py::is_operator())
      .def("__pow__", pybind11::overload_cast<double, const Measure &>(&measure_pow), py::is_operator())
      .def("__repr__", &Measure::repr);

  py::class_<RobotPose>(m, "RobotPose")
      .def(py::init<Eigen::Affine3d, std::optional<double>>(),
           "end_effector_pose"_a,
           "elbow_position"_a = std::nullopt)
      .def(py::init<const RobotPose &>()) // Copy constructor
      .def("__repr__", [](const RobotPose &robot_pose) {
        std::stringstream ss;
        ss << "(ee_pose=" << affine_to_str(robot_pose.end_effector_pose());
        if (robot_pose.elbow_position().has_value())
          ss << ", elbow=" << robot_pose.elbow_position().value();
        ss << ")";
        return ss.str();
      });

  py::class_<Affine>(m, "Affine")
      .def(py::init<const Eigen::Matrix<double, 4, 4> &>(),
           "transformation_matrix"_a = Eigen::Matrix<double, 4, 4>::Identity())
      .def(py::init<>([](const Vector<3> &translation, const Vector<4> &quaternion) {
        return Affine().fromPositionOrientationScale(
            translation, Eigen::Quaterniond(quaternion), Vector<3>::Ones());
      }))
      .def(py::init<const Affine &>()) // Copy constructor
      .def(py::self * py::self)
      .def_property_readonly("inverse", &Affine::inverse)
      .def_property_readonly("translation", [](const Affine &affine) {
        return affine.translation();
      })
      .def_property_readonly("quaternion", [](const Affine &affine) {
        return Eigen::Quaterniond(affine.rotation()).coeffs();
      })
      .def_property_readonly("matrix", [](const Affine &affine) {
        return affine.matrix();
      })
      .def("__repr__", affine_to_str);

  py::class_<Kinematics::NullSpaceHandling>(m, "NullSpaceHandling")
      .def(py::init<size_t, double>(), "joint_index"_a, "value"_a)
      .def_readwrite("joint_index", &Kinematics::NullSpaceHandling::joint_index)
      .def_readwrite("value", &Kinematics::NullSpaceHandling::value);

  py::class_<Kinematics>(m, "Kinematics")
      .def_static("forward", &Kinematics::forward, "q"_a)
      .def_static("forwardElbow", &Kinematics::forwardElbow, "q"_a)
      .def_static("forwardEuler", &Kinematics::forwardEuler, "q"_a)
      .def_static("jacobian", &Kinematics::jacobian, "q"_a)
      .def_static("inverse", &Kinematics::inverse, "target"_a, "q0"_a, "null_space"_a = std::nullopt);

  py::class_<franka::Duration>(m, "Duration")
      .def(py::init<>())
      .def(py::init<uint64_t>())
      .def("to_sec", &franka::Duration::toSec)
      .def("to_msec", &franka::Duration::toMSec)
      .def(py::self + py::self)
      .def(py::self += py::self)
      .def(py::self - py::self)
      .def(py::self -= py::self)
      .def(py::self * uint64_t())
      .def(py::self *= uint64_t())
      .def(py::self / uint64_t())
      .def(py::self /= uint64_t());

  py::class_<franka::Errors>(m, "Errors")
      .def(py::init<>())
      .def_property_readonly("joint_position_limits_violation",
                             [](const franka::Errors &e) { return e.joint_position_limits_violation; })
      .def_property_readonly("cartesian_position_limits_violation",
                             [](const franka::Errors &e) { return e.cartesian_position_limits_violation; })
      .def_property_readonly("self_collision_avoidance_violation",
                             [](const franka::Errors &e) { return e.self_collision_avoidance_violation; })
      .def_property_readonly("joint_velocity_violation",
                             [](const franka::Errors &e) { return e.joint_velocity_violation; })
      .def_property_readonly("cartesian_velocity_violation",
                             [](const franka::Errors &e) { return e.cartesian_velocity_violation; })
      .def_property_readonly("force_control_safety_violation",
                             [](const franka::Errors &e) { return e.force_control_safety_violation; })
      .def_property_readonly("joint_reflex", [](const franka::Errors &e) { return e.joint_reflex; })
      .def_property_readonly("cartesian_reflex", [](const franka::Errors &e) { return e.cartesian_reflex; })
      .def_property_readonly("max_goal_pose_deviation_violation",
                             [](const franka::Errors &e) { return e.max_goal_pose_deviation_violation; })
      .def_property_readonly("max_path_pose_deviation_violation",
                             [](const franka::Errors &e) { return e.max_path_pose_deviation_violation; })
      .def_property_readonly("cartesian_velocity_profile_safety_violation",
                             [](const franka::Errors &e) { return e.cartesian_velocity_profile_safety_violation; })
      .def_property_readonly("joint_position_motion_generator_start_pose_invalid",
                             [](const franka::Errors &e) { return e.joint_position_motion_generator_start_pose_invalid; })
      .def_property_readonly("joint_motion_generator_position_limits_violation",
                             [](const franka::Errors &e) { return e.joint_motion_generator_position_limits_violation; })
      .def_property_readonly("joint_motion_generator_velocity_limits_violation",
                             [](const franka::Errors &e) { return e.joint_motion_generator_velocity_limits_violation; })
      .def_property_readonly("joint_motion_generator_velocity_discontinuity",
                             [](const franka::Errors &e) { return e.joint_motion_generator_velocity_discontinuity; })
      .def_property_readonly("joint_motion_generator_acceleration_discontinuity",
                             [](const franka::Errors &e) { return e.joint_motion_generator_acceleration_discontinuity; })
      .def_property_readonly("cartesian_position_motion_generator_start_pose_invalid",
                             [](const franka::Errors &e) { return e.cartesian_position_motion_generator_start_pose_invalid; })
      .def_property_readonly("cartesian_motion_generator_elbow_limit_violation",
                             [](const franka::Errors &e) { return e.cartesian_motion_generator_elbow_limit_violation; })
      .def_property_readonly("cartesian_motion_generator_velocity_limits_violation",
                             [](const franka::Errors &e) { return e.cartesian_motion_generator_velocity_limits_violation; })
      .def_property_readonly("cartesian_motion_generator_velocity_discontinuity",
                             [](const franka::Errors &e) { return e.cartesian_motion_generator_velocity_discontinuity; })
      .def_property_readonly("cartesian_motion_generator_acceleration_discontinuity",
                             [](const franka::Errors &e) { return e.cartesian_motion_generator_acceleration_discontinuity; })
      .def_property_readonly("cartesian_motion_generator_elbow_sign_inconsistent",
                             [](const franka::Errors &e) { return e.cartesian_motion_generator_elbow_sign_inconsistent; })
      .def_property_readonly("cartesian_motion_generator_start_elbow_invalid",
                             [](const franka::Errors &e) { return e.cartesian_motion_generator_start_elbow_invalid; })
      .def_property_readonly("cartesian_motion_generator_joint_position_limits_violation",
                             [](const franka::Errors &e) { return e.cartesian_motion_generator_joint_position_limits_violation; })
      .def_property_readonly("cartesian_motion_generator_joint_velocity_limits_violation",
                             [](const franka::Errors &e) { return e.cartesian_motion_generator_joint_velocity_limits_violation; })
      .def_property_readonly("cartesian_motion_generator_joint_velocity_discontinuity",
                             [](const franka::Errors &e) { return e.cartesian_motion_generator_joint_velocity_discontinuity; })
      .def_property_readonly("cartesian_motion_generator_joint_acceleration_discontinuity",
                             [](const franka::Errors &e) { return e.cartesian_motion_generator_joint_acceleration_discontinuity; })
      .def_property_readonly("cartesian_position_motion_generator_invalid_frame",
                             [](const franka::Errors &e) { return e.cartesian_position_motion_generator_invalid_frame; })
      .def_property_readonly("force_controller_desired_force_tolerance_violation",
                             [](const franka::Errors &e) { return e.force_controller_desired_force_tolerance_violation; })
      .def_property_readonly("controller_torque_discontinuity",
                             [](const franka::Errors &e) { return e.controller_torque_discontinuity; })
      .def_property_readonly("start_elbow_sign_inconsistent",
                             [](const franka::Errors &e) { return e.start_elbow_sign_inconsistent; })
      .def_property_readonly("communication_constraints_violation",
                             [](const franka::Errors &e) { return e.communication_constraints_violation; })
      .def_property_readonly("power_limit_violation", [](const franka::Errors &e) { return e.power_limit_violation; })
      .def_property_readonly("joint_p2p_insufficient_torque_for_planning",
                             [](const franka::Errors &e) { return e.joint_p2p_insufficient_torque_for_planning; })
      .def_property_readonly("tau_j_range_violation", [](const franka::Errors &e) { return e.tau_j_range_violation; })
      .def_property_readonly("instability_detected", [](const franka::Errors &e) { return e.instability_detected; })
      .def_property_readonly("joint_move_in_wrong_direction",
                             [](const franka::Errors &e) { return e.joint_move_in_wrong_direction; });

  py::enum_<franka::RobotMode>(m, "RobotMode")
      .value("Other", franka::RobotMode::kOther)
      .value("Idle", franka::RobotMode::kIdle)
      .value("Move", franka::RobotMode::kMove)
      .value("Guiding", franka::RobotMode::kGuiding)
      .value("Reflex", franka::RobotMode::kReflex)
      .value("UserStopped", franka::RobotMode::kUserStopped)
      .value("AutomaticErrorRecovery", franka::RobotMode::kAutomaticErrorRecovery)
      .export_values();

  py::enum_<franka::ControllerMode>(m, "ControllerMode")
      .value("JointImpedance", franka::ControllerMode::kJointImpedance)
      .value("CartesianImpedance", franka::ControllerMode::kCartesianImpedance)
      .export_values();

  py::enum_<franka::RealtimeConfig>(m, "RealtimeConfig")
      .value("Enforce", franka::RealtimeConfig::kEnforce)
      .value("Ignore", franka::RealtimeConfig::kIgnore)
      .export_values();

  py::class_<franka::RobotState>(m, "RobotState")
      .def_readonly("O_T_EE", &franka::RobotState::O_T_EE)
      .def_readonly("O_T_EE_d", &franka::RobotState::O_T_EE_d)
      .def_readonly("F_T_EE", &franka::RobotState::F_T_EE)
      .def_readonly("EE_T_K", &franka::RobotState::EE_T_K)
      .def_readonly("m_ee", &franka::RobotState::m_ee)
      .def_readonly("I_ee", &franka::RobotState::I_ee)
      .def_readonly("F_x_Cee", &franka::RobotState::F_x_Cee)
      .def_readonly("m_load", &franka::RobotState::m_load)
      .def_readonly("I_load", &franka::RobotState::I_load)
      .def_readonly("F_x_Cload", &franka::RobotState::F_x_Cload)
      .def_readonly("m_total", &franka::RobotState::m_total)
      .def_readonly("I_total", &franka::RobotState::I_total)
      .def_readonly("F_x_Ctotal", &franka::RobotState::F_x_Ctotal)
      .def_readonly("elbow", &franka::RobotState::elbow)
      .def_readonly("elbow_d", &franka::RobotState::elbow_d)
      .def_readonly("elbow_c", &franka::RobotState::elbow_c)
      .def_readonly("delbow_c", &franka::RobotState::delbow_c)
      .def_readonly("ddelbow_c", &franka::RobotState::ddelbow_c)
      .def_readonly("tau_J", &franka::RobotState::tau_J)
      .def_readonly("tau_J_d", &franka::RobotState::tau_J_d)
      .def_readonly("dtau_J", &franka::RobotState::dtau_J)
      .def_readonly("q", &franka::RobotState::q)
      .def_readonly("q_d", &franka::RobotState::q_d)
      .def_readonly("dq", &franka::RobotState::dq)
      .def_readonly("dq_d", &franka::RobotState::dq_d)
      .def_readonly("ddq_d", &franka::RobotState::ddq_d)
      .def_readonly("joint_contact", &franka::RobotState::m_total)
      .def_readonly("cartesian_contact", &franka::RobotState::cartesian_contact)
      .def_readonly("joint_collision", &franka::RobotState::joint_collision)
      .def_readonly("cartesian_collision", &franka::RobotState::cartesian_collision)
      .def_readonly("tau_ext_hat_filtered", &franka::RobotState::tau_ext_hat_filtered)
      .def_readonly("O_F_ext_hat_K", &franka::RobotState::O_F_ext_hat_K)
      .def_readonly("K_F_ext_hat_K", &franka::RobotState::K_F_ext_hat_K)
      .def_readonly("O_T_EE_c", &franka::RobotState::O_T_EE_c)
      .def_readonly("O_dP_EE_c", &franka::RobotState::O_dP_EE_c)
      .def_readonly("O_ddP_EE_c", &franka::RobotState::O_ddP_EE_c)
      .def_readonly("theta", &franka::RobotState::theta)
      .def_readonly("dtheta", &franka::RobotState::dtheta)
      .def_readonly("current_errors", &franka::RobotState::current_errors)
      .def_readonly("last_motion_errors", &franka::RobotState::last_motion_errors)
      .def_readonly("control_command_success_rate", &franka::RobotState::control_command_success_rate)
      .def_readonly("robot_mode", &franka::RobotState::robot_mode)
      .def_readonly("time", &franka::RobotState::time);

  py::class_<franka::GripperState>(m, "GripperState")
      .def_readonly("width", &franka::GripperState::width)
      .def_readonly("max_width", &franka::GripperState::max_width)
      .def_readonly("is_grasped", &franka::GripperState::is_grasped)
      .def_readonly("temperature", &franka::GripperState::temperature)
      .def_readonly("time", &franka::GripperState::time);

  py::class_<Gripper>(m, "Gripper")
      .def(py::init<const std::string &, double, double>(), "fci_ip"_a, "speed"_a = 0.02, "force"_a = 20.0)
      .def_readwrite("gripper_force", &Gripper::gripper_force)
      .def_readwrite("gripper_speed", &Gripper::gripper_speed)
      .def_readonly("max_width", &Gripper::max_width)
      .def_readonly("has_error", &Gripper::has_error)
      .def("homing", &Gripper::homing, py::call_guard<py::gil_scoped_release>())
      .def("grasp",
           (bool (Gripper::*)(double, double, double, double, double)) &Gripper::grasp,
           py::call_guard<py::gil_scoped_release>())
      .def("move", (bool (Gripper::*)(double, double)) &Gripper::move, py::call_guard<py::gil_scoped_release>())
      .def("stop", &Gripper::stop)
      .def("read_once", &Gripper::readOnce)
      .def("server_version", &Gripper::serverVersion)
      .def("move", (bool (Gripper::*)(double)) &Gripper::move, py::call_guard<py::gil_scoped_release>())
      .def("move_unsafe", (bool (Gripper::*)(double)) &Gripper::move, py::call_guard<py::gil_scoped_release>())
      .def("width", &Gripper::width)
      .def("is_grasping", &Gripper::isGrasping)
      .def("open", &Gripper::open, py::call_guard<py::gil_scoped_release>())
      .def("clamp", (bool (Gripper::*)()) &Gripper::clamp, py::call_guard<py::gil_scoped_release>())
      .def("clamp", (bool (Gripper::*)(double)) &Gripper::clamp, py::call_guard<py::gil_scoped_release>())
      .def("release", (bool (Gripper::*)()) &Gripper::release, py::call_guard<py::gil_scoped_release>())
      .def("release", (bool (Gripper::*)(double)) &Gripper::release, py::call_guard<py::gil_scoped_release>())
      .def("releaseRelative", &Gripper::releaseRelative, py::call_guard<py::gil_scoped_release>())
      .def("get_state", &Gripper::get_state);

  py::register_exception<franka::CommandException>(m, "CommandException");
  py::register_exception<franka::ControlException>(m, "ControlException");
  py::register_exception<franka::IncompatibleVersionException>(m, "IncompatibleVersionException");
  py::register_exception<franka::InvalidOperationException>(m, "InvalidOperationException");
  py::register_exception<franka::ModelException>(m, "ModelException");
  py::register_exception<franka::NetworkException>(m, "NetworkException");
  py::register_exception<franka::ProtocolException>(m, "ProtocolException");
  py::register_exception<franka::RealtimeException>(m, "RealtimeException");
}
