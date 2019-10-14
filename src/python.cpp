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
    py::class_<Condition> condition(m, "Condition");
    condition.def(py::init<Condition::Axis, Condition::Comparison, double>())
        .def_readonly("has_fired", &Condition::has_fired);

    py::enum_<Condition::Axis>(condition, "Axis")
        .value("ForceZ", Condition::Axis::ForceZ)
        .value("ForceXYNorm", Condition::Axis::ForceXYNorm)
        .value("ForceXYZNorm", Condition::Axis::ForceXYZNorm)
        .export_values();

    py::enum_<Condition::Comparison>(condition, "Comparison")
        .value("Greater", Condition::Comparison::Greater)
        .value("Smaller", Condition::Comparison::Smaller)
        .export_values();

    py::class_<MotionData>(m, "MotionData")
        .def(py::init<>())
        .def_readwrite("velocity_rel", &MotionData::velocity_rel)
        .def_readwrite("acceleration_rel", &MotionData::acceleration_rel)
        .def("with_dynamics", &MotionData::withDynamics)
        .def("with_condition", &MotionData::withCondition);

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

    py::class_<Robot>(m, "Robot")
        .def(py::init<const std::string &>());
}