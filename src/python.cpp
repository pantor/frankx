#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>

#include <frankx/gripper.hpp>
#include <frankx/robot.hpp>

namespace py = pybind11;
using namespace pybind11::literals; // to bring in the `_a` literal
using namespace frankx;


PYBIND11_MODULE(frankx, m) {
    py::class_<Affine>(m, "Affine")
        .def(py::init<>())
        .def(py::init<double, double, double, double, double, double>(), "x"_a=0.0, "y"_a=0.0, "z"_a=0.0, "a"_a=0.0, "b"_a=0.0, "c"_a=0.0)
        .def(py::init<Vector6d>())
        .def(py::init<Vector7d>())
        .def(py::self * py::self)
        .def("matrix", &Affine::matrix)
        .def("inverse", &Affine::inverse)
        .def("is_approx", &Affine::isApprox)
        .def("translate", &Affine::translate)
        .def("pretranslate", &Affine::pretranslate)
        .def("translation", &Affine::translation)
        .def_property("x", &Affine::x, &Affine::set_x)
        .def_property("y", &Affine::y, &Affine::set_y)
        .def_property("z", &Affine::z, &Affine::set_z)
        .def("rotate", &Affine::rotate)
        .def("prerotate", &Affine::prerotate)
        .def("rotation", &Affine::rotation)
        .def_property("a", &Affine::a, &Affine::set_a)
        .def_property("b", &Affine::b, &Affine::set_b)
        .def_property("c", &Affine::c, &Affine::set_c)
        .def("get_inner_random", &Affine::getInnerRandom)
        .def("__repr__", &Affine::toString);

    py::class_<Condition> condition(m, "Condition");
    condition.def(py::init<Condition::Measure, Condition::Comparison, double>())
        .def(py::init<Condition::Measure, Condition::Comparison, double, std::shared_ptr<WaypointMotion>>())
        .def_readonly("has_fired", &Condition::has_fired)
        .def_readonly("has_action", &Condition::has_action);

    py::enum_<Condition::Measure>(condition, "Measure")
        .value("ForceZ", Condition::Measure::ForceZ)
        .value("ForceXYNorm", Condition::Measure::ForceXYNorm)
        .value("ForceXYZNorm", Condition::Measure::ForceXYZNorm)
        .value("Time", Condition::Measure::Time)
        .export_values();

    py::enum_<Condition::Comparison>(condition, "Comparison")
        .value("Greater", Condition::Comparison::Greater)
        .value("Smaller", Condition::Comparison::Smaller)
        .export_values();

    py::class_<MotionData>(m, "MotionData")
        .def(py::init<double>(), "dynamic_rel"_a = 1.0)
        .def_readwrite("velocity_rel", &MotionData::velocity_rel)
        .def_readwrite("acceleration_rel", &MotionData::acceleration_rel)
        .def_readonly("conditions", &MotionData::conditions)
        .def("with_dynamic_rel", &MotionData::withDynamicRel)
        .def("with_condition", &MotionData::withCondition);

    py::class_<Waypoint> waypoint(m, "Waypoint");
    py::enum_<Waypoint::ReferenceType>(waypoint, "Waypoint")
        .value("Absolute", Waypoint::ReferenceType::Absolute)
        .value("Relative", Waypoint::ReferenceType::Relative)
        .export_values();

    waypoint.def(py::init<>())
        .def(py::init<double>(), "minimum_time"_a)
        .def(py::init<const Affine &, double, Waypoint::ReferenceType, double>(), "affine"_a, "elbow"_a, "reference_type"_a = Waypoint::ReferenceType::Absolute, "dynamic_rel"_a = 1.0)
        .def(py::init<const Affine &, double, const Vector7d &, Waypoint::ReferenceType, double>(), "affine"_a, "elbow"_a, "velocity"_a, "reference_type"_a = Waypoint::ReferenceType::Absolute, "dynamic_rel"_a = 1.0)
        .def_readonly("reference_type", &Waypoint::reference_type)
        .def_readonly("minimum_time", &Waypoint::minimum_time)
        .def("get_target_affine", &Waypoint::getTargetAffine)
        .def("get_target_vector", &Waypoint::getTargetVector)
        .def("get_target_velocity", &Waypoint::getTargetVelocity);

    py::class_<JointMotion>(m, "JointMotion")
        .def(py::init<const std::array<double, 7>>());

    py::class_<WaypointMotion>(m, "WaypointMotion")
        .def(py::init<const std::vector<Waypoint> &>());

    py::class_<LinearMotion, WaypointMotion>(m, "LinearMotion")
        .def(py::init<const Affine&, double>());

    py::class_<LinearRelativeMotion, WaypointMotion>(m, "LinearRelativeMotion")
        .def(py::init<const Affine&>())
        .def(py::init<const Affine&, double>());

    py::class_<PositionHold, WaypointMotion>(m, "PositionHold")
        .def(py::init<double>());

    py::class_<franka::Errors>(m, "Errors")
        .def(py::init<>());

    py::class_<franka::RobotState>(m, "RobotState")
        .def_readonly("O_T_EE", &franka::RobotState::O_T_EE)
        .def_readonly("O_T_EE_d", &franka::RobotState::O_T_EE_d)
        .def_readonly("F_T_EE", &franka::RobotState::F_T_EE)
        .def_readonly("EE_T_K", &franka::RobotState::EE_T_K)
        .def_readonly("m_ee", &franka::RobotState::m_ee)
        .def_readonly("I_ee", &franka::RobotState::I_ee)
        .def_readonly("m_load", &franka::RobotState::m_load)
        .def_readonly("I_load", &franka::RobotState::I_load)
        .def_readonly("m_total", &franka::RobotState::m_total)
        .def_readonly("I_total", &franka::RobotState::I_total)
        .def_readonly("elbow", &franka::RobotState::elbow)
        .def_readonly("elbow_d", &franka::RobotState::elbow_d)
        .def_readonly("elbow_c", &franka::RobotState::elbow_c)
        .def_readonly("delbow_c", &franka::RobotState::delbow_c)
        .def_readonly("ddelbow_c", &franka::RobotState::ddelbow_c)
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
        .def_readonly("O_T_EE_c", &franka::RobotState::O_T_EE_c)
        .def_readonly("O_dP_EE_c", &franka::RobotState::O_dP_EE_c)
        .def_readonly("O_ddP_EE_c", &franka::RobotState::O_ddP_EE_c)
        .def_readonly("theta", &franka::RobotState::theta)
        .def_readonly("dtheta", &franka::RobotState::dtheta)
        .def_readonly("current_errors", &franka::RobotState::current_errors);

    py::class_<Robot>(m, "Robot")
        .def(py::init<const std::string &, double>(), "fci_ip"_a, "dynamic_rel"_a = 1.0)
        .def_readonly("max_translation_velocity", &Robot::max_translation_velocity)
        .def_readonly("max_rotation_velocity", &Robot::max_rotation_velocity)
        .def_readonly("max_elbow_velocity", &Robot::max_elbow_velocity)
        .def_readonly("max_translation_acceleration", &Robot::max_translation_acceleration)
        .def_readonly("max_rotation_acceleration", &Robot::max_rotation_acceleration)
        .def_readonly("max_elbow_acceleration", &Robot::max_elbow_acceleration)
        .def_readonly("max_translation_jerk", &Robot::max_translation_jerk)
        .def_readonly("max_rotation_jerk", &Robot::max_rotation_jerk)
        .def_readonly("max_elbow_jerk", &Robot::max_elbow_jerk)
        .def_readwrite("velocity_rel", &Robot::velocity_rel)
        .def_readwrite("acceleration_rel", &Robot::acceleration_rel)
        .def_readwrite("jerk_rel", &Robot::jerk_rel)
        .def("set_default_behavior", &Robot::setDefaultBehavior)
        .def("set_cartesian_impedance", &Robot::setCartesianImpedance)
        .def("set_dynamic_rel", &Robot::setDynamicRel)
        .def("has_errors", &Robot::hasErrors)
        .def("recover_from_errors", &Robot::recoverFromErrors)
        .def("automatic_error_recovery ", &Robot::automaticErrorRecovery)
        .def("read_once", &Robot::readOnce)
        .def("current_pose", &Robot::currentPose, "frame"_a = Affine())
        .def("stop", &Robot::stop)
        .def("move", (bool (Robot::*)(JointMotion)) &Robot::move)
        .def("move", (bool (Robot::*)(JointMotion, MotionData &)) &Robot::move)
        .def("move", (bool (Robot::*)(WaypointMotion)) &Robot::move)
        .def("move", (bool (Robot::*)(WaypointMotion, MotionData &)) &Robot::move);

    py::class_<franka::GripperState>(m, "GripperState")
        .def_readonly("width", &franka::GripperState::width)
        .def_readonly("max_width", &franka::GripperState::max_width)
        .def_readonly("is_grasped", &franka::GripperState::is_grasped)
        .def_readonly("temperature", &franka::GripperState::temperature);

    py::class_<Gripper>(m, "Gripper")
        .def(py::init<const std::string&, double>(), "fci_ip"_a, "gripper_speed"_a = 0.02)
        .def_readwrite("gripper_force", &Gripper::gripper_force)
        .def_readwrite("gripper_speed", &Gripper::gripper_speed)
        .def_readonly("max_width", &Gripper::max_width)
        .def("width", &Gripper::width)
        .def("homing", &Gripper::homing)
        .def("move", &Gripper::move)
        .def("stop", &Gripper::stop)
        // .def("read_once", &Gripper::readOnce)
        .def("is_grasping", &Gripper::isGrasping)
        .def("open", &Gripper::open)
        .def("clamp", &Gripper::clamp)
        .def("release", &Gripper::release);
}