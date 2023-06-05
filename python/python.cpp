#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <Python.h>

#include <franka/robot_state.h>

#include <franky.hpp>

#include "concurrent_queue.hpp"

namespace py = pybind11;
using namespace pybind11::literals; // to bring in the '_a' literal
using namespace franky;

ConcurrentQueue<std::function<void()>> callback_queue;

template<int dims>
std::string vecToStr(const Vector<dims> &vec) {
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

std::string affineToStr(const Affine &affine) {
  std::stringstream ss;
  ss << "(t=" << vecToStr(affine.translation().eval())
     << ", q=" << vecToStr(Eigen::Quaterniond(affine.rotation()).coeffs()) << ")";
  return ss.str();
}

template<typename ControlSignalType>
void mkMotionClass(py::module_ m, const std::string &control_signal_name) {
  py::class_<Motion<ControlSignalType>, std::shared_ptr<Motion<ControlSignalType>>>(
      m, (control_signal_name + "Motion").c_str())
      .def_property_readonly("reactions", &Motion<ControlSignalType>::reactions)
      .def("add_reaction", &Motion<ControlSignalType>::addReaction)
      .def("register_callback", [](
          Motion<ControlSignalType> &motion,
          const typename Motion<ControlSignalType>::CallbackType &callback
      ) {
        motion.registerCallback([callback](
            const franka::RobotState &robot_state,
            franka::Duration time_step,
            double rel_time,
            double abs_time,
            const ControlSignalType &control_signal) {
          callback_queue.push([callback, robot_state, time_step, rel_time, abs_time, control_signal]() {
            callback(robot_state, time_step, rel_time, abs_time, control_signal);
          });
        });
      }, "callback"_a);
}

template<typename ControlSignalType>
void mkReactionClass(py::module_ m, const std::string &control_signal_name) {
  py::class_<Reaction<ControlSignalType>, std::shared_ptr<Reaction<ControlSignalType>>>(
      m, (control_signal_name + "Reaction").c_str())
      .def(py::init<const Condition &, std::shared_ptr<Motion<ControlSignalType>>>(),
           "condition"_a, "motion"_a = nullptr)
      .def("register_callback", [](
          Reaction<ControlSignalType> &reaction,
          const std::function<void(const franka::RobotState &, double, double)> &callback
      ) {
        reaction.registerCallback([callback](
            const franka::RobotState &robot_state, double rel_time, double abs_time) {
          callback_queue.push([callback, robot_state, rel_time, abs_time]() {
            callback(robot_state, rel_time, abs_time);
          });
        });
      }, "callback"_a);
}

[[noreturn]] void executeCallbacks() {
  while (true) {
    std::function<void()> callback = callback_queue.pop();
    callback();
  }
}

template<typename ControlSignalType>
void robotMove(Robot &robot, std::shared_ptr<Motion<ControlSignalType>> motion, bool async) {
  robot.move(motion, true);
  if (!async) {
    auto future = std::async(std::launch::async, &Robot::joinMotion, &robot);
    // Check if python wants to terminate every 100 ms
    while (future.wait_for(std::chrono::milliseconds(100)) == std::future_status::timeout) {
      bool terminate;
      {
        auto gstate = PyGILState_Ensure();
        terminate = Py_IsInitialized() && PyErr_CheckSignals() == -1;
        PyGILState_Release(gstate);
      }
      if (terminate) {
        robot.stop();
        future.wait();
      }
    }
  }
}
// Start the callback execution thread
std::thread callback_thread(executeCallbacks);

PYBIND11_MODULE(_franky, m) {
  m.doc() = "High-Level Motion Library for the Franka Panda Robot";

  py::class_<Measure>(m, "Measure")
      .def_property_readonly_static("FORCE_X", [](py::object) { return Measure::ForceX(); })
      .def_property_readonly_static("FORCE_Y", [](py::object) { return Measure::ForceY(); })
      .def_property_readonly_static("FORCE_Z", [](py::object) { return Measure::ForceZ(); })
      .def_property_readonly_static("REL_TIME", [](py::object) { return Measure::RelTime(); })
      .def_property_readonly_static("ABS_TIME", [](py::object) { return Measure::AbsTime(); })
      .def("__eq__", py::overload_cast<const Measure &, const Measure &>(&operator==), py::is_operator())
      .def("__eq__", py::overload_cast<const Measure &, double>(&operator==), py::is_operator())
      .def("__ne__", py::overload_cast<const Measure &, const Measure &>(&operator!=), py::is_operator())
      .def("__ne__", py::overload_cast<const Measure &, double>(&operator!=), py::is_operator())
      .def("__gt__", py::overload_cast<const Measure &, const Measure &>(&operator>), py::is_operator())
      .def("__gt__", py::overload_cast<const Measure &, double>(&operator>), py::is_operator())
      .def("__ge__", py::overload_cast<const Measure &, const Measure &>(&operator>=), py::is_operator())
      .def("__ge__", py::overload_cast<const Measure &, double>(&operator>=), py::is_operator())
      .def("__lt__", py::overload_cast<const Measure &, const Measure &>(&operator<), py::is_operator())
      .def("__lt__", py::overload_cast<const Measure &, double>(&operator<), py::is_operator())
      .def("__le__", py::overload_cast<const Measure &, const Measure &>(&operator<=), py::is_operator())
      .def("__le__", py::overload_cast<const Measure &, double>(&operator<=), py::is_operator())
      .def("__add__", py::overload_cast<const Measure &, const Measure &>(&operator+), py::is_operator())
      .def("__add__", py::overload_cast<const Measure &, double>(&operator+), py::is_operator())
      .def("__radd__", [](const Measure &measure, double constant) { return constant + measure; }, py::is_operator())
      .def("__sub__", py::overload_cast<const Measure &, const Measure &>(&operator-), py::is_operator())
      .def("__sub__", py::overload_cast<const Measure &, double>(&operator-), py::is_operator())
      .def("__rsub__", [](const Measure &measure, double constant) { return constant - measure; }, py::is_operator())
      .def("__mul__", py::overload_cast<const Measure &, const Measure &>(&operator*), py::is_operator())
      .def("__mul__", py::overload_cast<const Measure &, double>(&operator*), py::is_operator())
      .def("__rmul__", [](const Measure &measure, double constant) { return constant * measure; }, py::is_operator())
      .def("__div__", py::overload_cast<const Measure &, const Measure &>(&operator/), py::is_operator())
      .def("__div__", py::overload_cast<const Measure &, double>(&operator/), py::is_operator())
      .def("__rdiv__", [](const Measure &measure, double constant) { return constant / measure; }, py::is_operator())
      .def("__pow__", py::overload_cast<const Measure &, const Measure &>(&measure_pow), py::is_operator())
      .def("__pow__", py::overload_cast<const Measure &, double>(&measure_pow), py::is_operator())
      .def("__rpow__",
           [](const Measure &measure, double constant) { return measure_pow(constant, measure); },
           py::is_operator())
      .def("__repr__", &Measure::repr);

  py::class_<Condition>(m, "Condition")
      .def("__repr__", &Condition::repr)
      .def_property_readonly_static("TRUE", [](py::object) { return Condition::TRUE(); })
      .def_property_readonly_static("FALSE", [](py::object) { return Condition::FALSE(); });

  mkMotionClass<franka::Torques>(m, "Torque");
  mkMotionClass<franka::JointVelocities>(m, "JointVelocity");
  mkMotionClass<franka::JointPositions>(m, "JointPosition");
  mkMotionClass<franka::CartesianVelocities>(m, "CartesianVelocity");
  mkMotionClass<franka::CartesianPose>(m, "CartesianPose");

  py::class_<ImpedanceMotion, Motion<franka::Torques>, std::shared_ptr<ImpedanceMotion>>(m, "ImpedanceMotion");

  py::enum_<ReferenceType>(m, "ReferenceType")
      .value("RELATIVE", ReferenceType::Relative)
      .value("ABSOLUTE", ReferenceType::Absolute)
      .export_values();

  py::class_<ExponentialImpedanceMotion, ImpedanceMotion, std::shared_ptr<ExponentialImpedanceMotion>>(
      m, "ExponentialImpedanceMotion")
      .def(py::init<>([](
               const Affine &target, ReferenceType target_type, double translational_stiffness,
               double rotational_stiffness, std::optional<std::array<std::optional<double>, 6>> force_constraints,
               double exponential_decay = 0.005) {
             Eigen::Vector<bool, 6> force_constraints_active = Eigen::Vector<bool, 6>::Zero();
             Eigen::Vector<double, 6> force_constraints_value;
             if (force_constraints.has_value()) {
               for (int i = 0; i < 6; i++) {
                 force_constraints_value[i] = force_constraints.value()[i].value_or(NAN);
                 force_constraints_active[i] = force_constraints.value()[i].has_value();
               }
             }
             return std::make_shared<ExponentialImpedanceMotion>(
                 target,
                 ExponentialImpedanceMotion::Params{
                     target_type, translational_stiffness, rotational_stiffness, force_constraints_value,
                     force_constraints_active, exponential_decay});
           }),
           "target"_a,
           "target_type"_a = ReferenceType::Absolute,
           "translational_stiffness"_a = 2000,
           "rotational_stiffness"_a = 200,
           "force_constraints"_a = std::nullopt,
           "exponential_decay"_a = 0.005);

  py::class_<LinearImpedanceMotion, ImpedanceMotion, std::shared_ptr<LinearImpedanceMotion>>(m, "LinearImpedanceMotion")
      .def(py::init<>([](
               const Affine &target,
               double duration,
               ReferenceType target_type,
               double translational_stiffness,
               double rotational_stiffness,
               std::optional<std::array<std::optional<double>, 6>> force_constraints,
               bool return_when_finished,
               double finish_wait_factor) {
             Eigen::Vector<bool, 6> force_constraints_active = Eigen::Vector<bool, 6>::Zero();
             Eigen::Vector<double, 6> force_constraints_value;
             if (force_constraints.has_value()) {
               for (int i = 0; i < 6; i++) {
                 force_constraints_value[i] = force_constraints.value()[i].value_or(NAN);
                 force_constraints_active[i] = force_constraints.value()[i].has_value();
               }
             }
             return std::make_shared<LinearImpedanceMotion>(
                 target, duration,
                 LinearImpedanceMotion::Params{
                     target_type, translational_stiffness, rotational_stiffness, force_constraints_value,
                     force_constraints_active, return_when_finished, finish_wait_factor});
           }),
           "target"_a,
           "duration"_a,
           "target_type"_a = ReferenceType::Absolute,
           "translational_stiffness"_a = 2000,
           "rotational_stiffness"_a = 200,
           "force_constraints"_a = std::nullopt,
           "return_when_finished"_a = true,
           "finish_wait_factor"_a = 1.2);

  py::class_<Waypoint<Vector7d>>(m, "JointWaypoint")
      .def(py::init<>(
               [](
                   const Vector7d &target,
                   ReferenceType reference_type,
                   double velocity_rel,
                   double acceleration_rel,
                   double jerk_rel,
                   bool max_dynamics,
                   std::optional<double> minimum_time) {
                 return Waypoint<Vector7d>{
                     target, reference_type, velocity_rel, acceleration_rel, jerk_rel, max_dynamics, minimum_time};
               }
           ),
           "target"_a,
           "reference_type"_a = ReferenceType::Absolute,
           "velocity_rel"_a = 1.0,
           "acceleration_rel"_a = 1.0,
           "jerk_rel"_a = 1.0,
           "max_dynamics"_a = false,
           "minimum_time"_a = std::nullopt)
      .def_readonly("target", &Waypoint<Vector7d>::target)
      .def_readonly("reference_type", &Waypoint<Vector7d>::reference_type)
      .def_readonly("velocity_rel", &Waypoint<Vector7d>::velocity_rel)
      .def_readonly("acceleration_rel", &Waypoint<Vector7d>::acceleration_rel)
      .def_readonly("jerk_rel", &Waypoint<Vector7d>::jerk_rel)
      .def_readonly("max_dynamics", &Waypoint<Vector7d>::max_dynamics)
      .def_readonly("minimum_time", &Waypoint<Vector7d>::minimum_time);

  py::class_<JointWaypointMotion, Motion<franka::JointPositions>, std::shared_ptr<JointWaypointMotion>>(
      m, "JointWaypointMotion")
      .def(py::init<>([](
               const std::vector<Waypoint<Vector7d>> &waypoints, double velocity_rel, double acceleration_rel,
               double jerk_rel, bool return_when_finished) {
             return std::make_shared<JointWaypointMotion>(
                 waypoints,
                 JointWaypointMotion::Params{
                     velocity_rel, acceleration_rel, jerk_rel, return_when_finished});
           }),
           "waypoints"_a,
           "velocity_rel"_a = 1.0,
           "acceleration_rel"_a = 1.0,
           "jerk_rel"_a = 1.0,
           "return_when_finished"_a = true);

  py::class_<CartesianWaypointMotion, Motion<franka::CartesianPose>, std::shared_ptr<CartesianWaypointMotion>>(
      m, "CartesianWaypointMotion")
      .def(py::init<>([](
               const std::vector<Waypoint<RobotPose>> &waypoints,
               const std::optional<Affine> &frame = std::nullopt,
               double velocity_rel = 1.0,
               double acceleration_rel = 1.0,
               double jerk_rel = 1.0,
               bool max_dynamics = false,
               bool return_when_finished = true) {
             return std::make_shared<CartesianWaypointMotion>(
                 waypoints,
                 CartesianWaypointMotion::Params{
                     {velocity_rel, acceleration_rel, jerk_rel, max_dynamics, return_when_finished},
                     frame.value_or(Affine::Identity())});
           }),
           "waypoints"_a,
           "frame"_a = std::nullopt,
           "velocity_rel"_a = 1.0,
           "acceleration_rel"_a = 1.0,
           "jerk_rel"_a = 1.0,
           "max_dynamics"_a = false,
           "return_when_finished"_a = true);

  py::class_<LinearMotion, CartesianWaypointMotion, std::shared_ptr<LinearMotion>>(m, "LinearMotion")
      .def(py::init<>([](
               const RobotPose &target,
               ReferenceType reference_type,
               const std::optional<Affine> &frame,
               double velocity_rel,
               double acceleration_rel,
               double jerk_rel,
               bool return_when_finished) {
             return std::make_shared<LinearMotion>(
                 target,
                 reference_type,
                 frame.value_or(Affine::Identity()),
                 velocity_rel,
                 acceleration_rel,
                 jerk_rel,
                 return_when_finished);
           }),
           "target"_a,
           "reference_type"_a = ReferenceType::Absolute,
           "frame"_a = std::nullopt,
           "velocity_rel"_a = 1.0,
           "acceleration_rel"_a = 1.0,
           "jerk_rel"_a = 1.0,
           "return_when_finished"_a = true);

  py::class_<StopMotion<franka::CartesianPose>,
             Motion<franka::CartesianPose>,
             std::shared_ptr<StopMotion<franka::CartesianPose>>>(m, "CartesianStopMotion")
      .def(py::init<>());

  py::class_<StopMotion<franka::JointPositions>,
             Motion<franka::JointPositions>,
             std::shared_ptr<StopMotion<franka::JointPositions>>>(m, "JointStopMotion")
      .def(py::init<>());

  mkReactionClass<franka::Torques>(m, "Torque");
  mkReactionClass<franka::JointVelocities>(m, "JointVelocity");
  mkReactionClass<franka::JointPositions>(m, "JointPosition");
  mkReactionClass<franka::CartesianVelocities>(m, "CartesianVelocity");
  mkReactionClass<franka::CartesianPose>(m, "CartesianPose");

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

  py::class_<Kinematics>(m, "Kinematics")
      .def_static("forward", &Kinematics::forward, "q"_a)
      .def_static("forward_elbow", &Kinematics::forwardElbow, "q"_a)
      .def_static("forward_euler", &Kinematics::forwardEuler, "q"_a)
      .def_static("jacobian", &Kinematics::jacobian, "q"_a)
      .def_static("inverse", &Kinematics::inverse, "target"_a, "q0"_a, "null_space"_a = std::nullopt);

  py::enum_<franka::ControllerMode>(m, "ControllerMode")
      .value("JointImpedance", franka::ControllerMode::kJointImpedance)
      .value("CartesianImpedance", franka::ControllerMode::kCartesianImpedance)
      .export_values();

  py::enum_<franka::RealtimeConfig>(m, "RealtimeConfig")
      .value("Enforce", franka::RealtimeConfig::kEnforce)
      .value("Ignore", franka::RealtimeConfig::kIgnore)
      .export_values();

  py::class_<Robot>(m, "Robot")
      .def(py::init<>([](
               const std::string &fci_ip,
               double velocity_rel,
               double acceleration_rel,
               double jerk_rel,
               double default_torque_threshold,
               double default_force_threshold,
               franka::ControllerMode controller_mode,
               franka::RealtimeConfig realtime_config) {
             return std::make_unique<Robot>(
                 fci_ip, Robot::Params{
                     velocity_rel, acceleration_rel, jerk_rel, default_torque_threshold, default_force_threshold,
                     controller_mode, realtime_config});
           }),
           "fci_ip"_a,
           "velocity_rel"_a = 1.0,
           "acceleration_rel"_a = 1.0,
           "jerk_rel"_a = 1.0,
           "default_torque_threshold"_a = 20.0,
           "default_force_threshold"_a = 30.0,
           "controller_mode"_a = franka::ControllerMode::kJointImpedance,
           "realtime_config"_a = franka::RealtimeConfig::kEnforce)
      .def("set_dynamic_rel", py::overload_cast<double>(&Robot::setDynamicRel), "dynamic_rel"_a)
      .def("set_dynamic_rel", py::overload_cast<double, double, double>(&Robot::setDynamicRel),
           "velocity_rel"_a, "acceleration_rel"_a, "jerk_rel"_a)
      .def("recover_from_errors", &Robot::recoverFromErrors)
      .def("move", &robotMove<franka::CartesianPose>, "motion"_a, "asynchronous"_a = false,
           py::call_guard<py::gil_scoped_release>())
      .def("move", &robotMove<franka::CartesianVelocities>, "motion"_a, "asynchronous"_a = false,
           py::call_guard<py::gil_scoped_release>())
      .def("move", &robotMove<franka::JointPositions>, "motion"_a, "asynchronous"_a = false,
           py::call_guard<py::gil_scoped_release>())
      .def("move", &robotMove<franka::JointVelocities>, "motion"_a, "asynchronous"_a = false,
           py::call_guard<py::gil_scoped_release>())
      .def("move", &robotMove<franka::Torques>, "motion"_a, "asynchronous"_a = false,
           py::call_guard<py::gil_scoped_release>())
      .def("join_motion", &Robot::joinMotion, py::call_guard<py::gil_scoped_release>())
      .def("set_collision_behavior", py::overload_cast<double, double>(&Robot::setCollisionBehavior),
           "torque_threshold"_a, "force_threshold"_a)
      .def("set_collision_behavior",
           py::overload_cast<
               const std::array<double, 7> &,
               const std::array<double, 6> &>(&Robot::setCollisionBehavior),
           "torque_thresholds"_a, "force_thresholds"_a)
      .def("set_collision_behavior", py::overload_cast<double, double, double, double>(&Robot::setCollisionBehavior),
           "lower_torque_threshold"_a, "upper_torque_threshold"_a, "lower_force_threshold"_a, "upper_force_threshold"_a)
      .def("set_collision_behavior",
           py::overload_cast<
               const std::array<double, 7> &,
               const std::array<double, 7> &,
               const std::array<double, 6> &,
               const std::array<double, 6> &
           >(&Robot::setCollisionBehavior),
           "lower_torque_thresholds"_a,
           "upper_torque_thresholds"_a,
           "lower_force_thresholds"_a,
           "upper_force_thresholds"_a)
      .def("set_collision_behavior",
           py::overload_cast<
               double,
               double,
               double,
               double,
               double,
               double,
               double,
               double>(&Robot::setCollisionBehavior),
           "lower_torque_threshold_acceleration"_a,
           "upper_torque_threshold_acceleration"_a,
           "lower_torque_threshold_nominal"_a,
           "upper_torque_threshold_nominal"_a,
           "lower_force_threshold_acceleration"_a,
           "upper_force_threshold_acceleration"_a,
           "lower_force_threshold_nominal"_a,
           "upper_force_threshold_nominal"_a)
      .def("set_collision_behavior",
           py::overload_cast<
               const std::array<double, 7> &,
               const std::array<double, 7> &,
               const std::array<double, 7> &,
               const std::array<double, 7> &,
               const std::array<double, 6> &,
               const std::array<double, 6> &,
               const std::array<double, 6> &,
               const std::array<double, 6> &
           >(&Robot::setCollisionBehavior),
           "lower_torque_thresholds_acceleration"_a,
           "upper_torque_thresholds_acceleration"_a,
           "lower_torque_thresholds_nominal"_a,
           "upper_torque_thresholds_nominal"_a,
           "lower_force_thresholds_acceleration"_a,
           "upper_force_thresholds_acceleration"_a,
           "lower_force_thresholds_nominal"_a,
           "upper_force_thresholds_nominal"_a)
      .def("set_joint_impedance", &Robot::setJointImpedance, "K_theta"_a)
      .def("set_cartesian_impedance", &Robot::setCartesianImpedance, "K_x"_a)
      .def("set_guiding_mode", &Robot::setGuidingMode, "guiding_mode"_a, "elbow"_a)
      .def("set_k", &Robot::setK, "EE_T_K"_a)
      .def("set_ee", &Robot::setEE, "NE_T_EE"_a)
      .def("set_load", &Robot::setLoad, "load_mass"_a, "F_x_Cload"_a, "load_inertia"_a)
      .def("stop", &Robot::stop)
      .def_property_readonly("velocity_rel", &Robot::velocity_rel)
      .def_property_readonly("acceleration_rel", &Robot::acceleration_rel)
      .def_property_readonly("jerk_rel", &Robot::jerk_rel)
      .def_property_readonly("has_errors", &Robot::hasErrors)
      .def_property_readonly("current_pose", &Robot::currentPose)
      .def_property_readonly("current_joint_positions", &Robot::currentJointPositions)
      .def_property_readonly("state", &Robot::state)
      .def_property_readonly("is_in_control", &Robot::is_in_control)
      .def_readonly_static("max_translation_velocity", &Robot::max_translation_velocity, "[m/s]")
      .def_readonly_static("max_rotation_velocity", &Robot::max_rotation_velocity, "[rad/s]")
      .def_readonly_static("max_elbow_velocity", &Robot::max_elbow_velocity, "[rad/s]")
      .def_readonly_static("max_translation_acceleration", &Robot::max_translation_acceleration, "[m/s²]")
      .def_readonly_static("max_rotation_acceleration", &Robot::max_rotation_acceleration, "[rad/s²]")
      .def_readonly_static("max_elbow_acceleration", &Robot::max_elbow_acceleration, "[rad/s²]")
      .def_readonly_static("max_translation_jerk", &Robot::max_translation_jerk, "[m/s³]")
      .def_readonly_static("max_rotation_jerk", &Robot::max_rotation_jerk, "[rad/s³]")
      .def_readonly_static("max_elbow_jerk", &Robot::max_elbow_jerk, "[rad/s³]")
      .def_readonly_static("degrees_of_freedom", &Robot::degrees_of_freedoms)
      .def_readonly_static("control_rate", &Robot::control_rate, "[s]")
      .def_property_readonly_static("max_joint_velocity", []() {
        return Vector7d::Map(Robot::max_joint_velocity.data());
      }, "[rad/s]")
      .def_property_readonly_static("max_joint_acceleration", []() {
        return Vector7d::Map(Robot::max_joint_acceleration.data());
      }, "[rad/s²]")
      .def_property_readonly_static("max_joint_jerk", []() {
        return Vector7d::Map(Robot::max_joint_jerk.data());
      }, "[rad/s^3]")
      .def_static("forward_kinematics", &Robot::forwardKinematics, "q"_a)
      .def_static("inverseKinematics", &Robot::inverseKinematics, "target"_a, "q0"_a);

  py::class_<RobotPose>(m, "RobotPose")
      .def(py::init<Affine, std::optional<double>>(),
           "end_effector_pose"_a,
           "elbow_position"_a = std::nullopt)
      .def(py::init<const RobotPose &>()) // Copy constructor
      .def("with_elbow_position", &RobotPose::with_elbow_position, "elbow_position"_a)
      .def_property_readonly("end_effector_pose", &RobotPose::end_effector_pose)
      .def_property_readonly("elbow_position", &RobotPose::elbow_position)
      .def("__mul__", py::overload_cast<const RobotPose &, const Affine &>(&operator*), py::is_operator())
      .def("__rmul__",
           [](const RobotPose &robot_pose, const Affine &affine) { return affine * robot_pose; },
           py::is_operator())
      .def("__repr__", [](const RobotPose &robot_pose) {
        std::stringstream ss;
        ss << "(ee_pose=" << affineToStr(robot_pose.end_effector_pose());
        if (robot_pose.elbow_position().has_value())
          ss << ", elbow=" << robot_pose.elbow_position().value();
        ss << ")";
        return ss.str();
      });
  py::implicitly_convertible<Affine, RobotPose>();

  py::class_<Waypoint<RobotPose>>(m, "CartesianWaypoint")
      .def(py::init<>(
               [](
                   const RobotPose &robot_pose,
                   ReferenceType reference_type,
                   double velocity_rel,
                   double acceleration_rel,
                   double jerk_rel,
                   bool max_dynamics,
                   std::optional<double> minimum_time) {
                 return Waypoint<RobotPose>{
                     robot_pose, reference_type, velocity_rel, acceleration_rel, jerk_rel, max_dynamics, minimum_time};
               }
           ),
           "robot_pose"_a,
           "reference_type"_a = ReferenceType::Absolute,
           "velocity_rel"_a = 1.0,
           "acceleration_rel"_a = 1.0,
           "jerk_rel"_a = 1.0,
           "max_dynamics"_a = false,
           "minimum_time"_a = std::nullopt)
      .def_readonly("target", &Waypoint<RobotPose>::target)
      .def_readonly("reference_type", &Waypoint<RobotPose>::reference_type)
      .def_readonly("velocity_rel", &Waypoint<RobotPose>::velocity_rel)
      .def_readonly("acceleration_rel", &Waypoint<RobotPose>::acceleration_rel)
      .def_readonly("jerk_rel", &Waypoint<RobotPose>::jerk_rel)
      .def_readonly("max_dynamics", &Waypoint<RobotPose>::max_dynamics)
      .def_readonly("minimum_time", &Waypoint<RobotPose>::minimum_time);

  py::class_<Affine>(m, "Affine")
      .def(py::init<const Eigen::Matrix<double, 4, 4> &>(),
           "transformation_matrix"_a = Eigen::Matrix<double, 4, 4>::Identity())
      .def(py::init<>([](const Vector<3> &translation, const Vector<4> &quaternion) {
        return Affine().fromPositionOrientationScale(
            translation, Eigen::Quaterniond(quaternion), Vector<3>::Ones());
      }), "translation"_a = Vector<3>{0, 0, 0}, "quaternion"_a = Vector<4>{0, 0, 0, 1})
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
      .def("__repr__", &affineToStr);

  py::class_<Kinematics::NullSpaceHandling>(m, "NullSpaceHandling")
      .def(py::init<size_t, double>(), "joint_index"_a, "value"_a)
      .def_readwrite("joint_index", &Kinematics::NullSpaceHandling::joint_index)
      .def_readwrite("value", &Kinematics::NullSpaceHandling::value);

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
      .def_property_readonly("joint_reflex",
                             [](const franka::Errors &e) { return e.joint_reflex; })
      .def_property_readonly("cartesian_reflex",
                             [](const franka::Errors &e) { return e.cartesian_reflex; })
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
      .def_property_readonly("power_limit_violation",
                             [](const franka::Errors &e) { return e.power_limit_violation; })
      .def_property_readonly("joint_p2p_insufficient_torque_for_planning",
                             [](const franka::Errors &e) { return e.joint_p2p_insufficient_torque_for_planning; })
      .def_property_readonly("tau_j_range_violation",
                             [](const franka::Errors &e) { return e.tau_j_range_violation; })
      .def_property_readonly("instability_detected",
                             [](const franka::Errors &e) { return e.instability_detected; })
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

  py::class_<franka::Torques>(m, "Torques")
      .def_readonly("tau_J", &franka::Torques::tau_J);

  py::class_<franka::JointVelocities>(m, "JointVelocities")
      .def_readonly("dq", &franka::JointVelocities::dq);

  py::class_<franka::JointPositions>(m, "JointPositions")
      .def_readonly("q", &franka::JointPositions::q);

  py::class_<franka::CartesianVelocities>(m, "CartesianVelocities")
      .def_readonly("O_dP_EE", &franka::CartesianVelocities::O_dP_EE);

  py::class_<franka::CartesianPose>(m, "CartesianPose")
      .def_readonly("O_T_EE", &franka::CartesianPose::O_T_EE);

  py::register_exception<franka::CommandException>(m, "CommandException");
  py::register_exception<franka::ControlException>(m, "ControlException");
  py::register_exception<franka::IncompatibleVersionException>(m, "IncompatibleVersionException");
  py::register_exception<franka::InvalidOperationException>(m, "InvalidOperationException");
  py::register_exception<franka::ModelException>(m, "ModelException");
  py::register_exception<franka::NetworkException>(m, "NetworkException");
  py::register_exception<franka::ProtocolException>(m, "ProtocolException");
  py::register_exception<franka::RealtimeException>(m, "RealtimeException");
  py::register_exception<franky::InvalidMotionTypeException>(m, "InvalidMotionTypeException");
  py::register_exception<franky::ReactionRecursionException>(m, "ReactionRecursionException");
  py::register_exception<franky::GripperException>(m, "GripperException");
}
