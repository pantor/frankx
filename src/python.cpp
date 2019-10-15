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
        .def(py::init<>())
        .def_readwrite("velocity_rel", &MotionData::velocity_rel)
        .def_readwrite("acceleration_rel", &MotionData::acceleration_rel)
        .def_readonly("conditions", &MotionData::conditions)
        .def("with_dynamics", &MotionData::withDynamics)
        .def("with_condition", &MotionData::withCondition);

    py::class_<Waypoint> waypoint(m, "Waypoint");
    waypoint.def(py::init<>())
        .def(py::init<double>())
        .def(py::init<const Affine &, double>())
        .def(py::init<const Affine &, double, Waypoint::ReferenceType>())
        .def(py::init<const Affine &, double, const std::array<double, 7> &>())
        .def(py::init<const Affine &, double, const std::array<double, 7> &, Waypoint::ReferenceType>())
        .def_readonly("reference_type", &Waypoint::reference_type)
        .def_readonly("minimum_time", &Waypoint::minimum_time)
        .def("get_target_affine", &Waypoint::getTargetAffine)
        .def("get_target_vector", &Waypoint::getTargetVector)
        .def("get_target_velocity", &Waypoint::getTargetVelocity);

    py::enum_<Waypoint::ReferenceType>(waypoint, "Waypoint")
        .value("Absolute", Waypoint::ReferenceType::Absolute)
        .value("Relative", Waypoint::ReferenceType::Relative)
        .export_values();

    py::class_<JointMotion>(m, "JointMotion")
        .def(py::init<double, const std::array<double, 7>>());

    py::class_<WaypointMotion>(m, "WaypointMotion")
        .def(py::init<const std::vector<Waypoint> &>());

    py::class_<LinearMotion, WaypointMotion>(m, "LinearMotion")
        .def(py::init<const Affine&, double>());

    py::class_<LinearRelativeMotion, WaypointMotion>(m, "LinearRelativeMotion")
        .def(py::init<const Affine&>())
        .def(py::init<const Affine&, double>());

    py::class_<PositionHold, WaypointMotion>(m, "PositionHold")
        .def(py::init<double>());

    py::class_<Robot>(m, "Robot")
        .def(py::init<const std::string &>())
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
        .def("set_default", &Robot::setDefault)
        .def("set_dynamic_rel", &Robot::setDynamicRel)
        .def("move", (bool (Robot::*)(const JointMotion &)) &Robot::move)
        .def("move", (bool (Robot::*)(const WaypointMotion &)) &Robot::move)
        .def("move", (bool (Robot::*)(const WaypointMotion &, MotionData &)) &Robot::move);

    py::class_<Gripper>(m, "Gripper")
        .def(py::init<const std::string&>())
        .def(py::init<const std::string&, double>())
        .def_readwrite("gripper_force", &Gripper::gripper_force)
        .def_readwrite("gripper_speed", &Gripper::gripper_speed)
        .def_readonly("max_width", &Gripper::max_width)
        .def("width", &Gripper::width)
        .def("stop", &Gripper::stop)
        .def("homing", &Gripper::homing)
        .def("is_grasping", &Gripper::isGrasping)
        .def("move", &Gripper::move)
        .def("open", &Gripper::open)
        .def("clamp", &Gripper::clamp)
        .def("release", &Gripper::release);
}