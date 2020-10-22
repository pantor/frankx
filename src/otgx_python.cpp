#include <array>
#include <string>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>

#include <otgx/quintic.hpp>

namespace py = pybind11;
using namespace pybind11::literals; // to bring in the `_a` literal
using namespace otgx;


PYBIND11_MODULE(otgx, m) {
    m.doc() = "Online Trajectory Generation";

    py::class_<InputParameter<1>>(m, "InputParameter")
        .def(py::init<>())
        .def_readwrite("current_position", &InputParameter<1>::current_position)
        .def_readwrite("current_velocity", &InputParameter<1>::current_velocity)
        .def_readwrite("current_acceleration", &InputParameter<1>::current_acceleration)
        .def_readwrite("target_position", &InputParameter<1>::target_position)
        .def_readwrite("target_velocity", &InputParameter<1>::target_velocity)
        .def_readwrite("target_acceleration", &InputParameter<1>::target_acceleration)
        .def_readwrite("max_velocity", &InputParameter<1>::max_velocity)
        .def_readwrite("max_acceleration", &InputParameter<1>::max_acceleration)
        .def_readwrite("max_jerk", &InputParameter<1>::max_jerk);

    py::class_<OutputParameter<1>>(m, "OutputParameter")
        .def(py::init<>())
        .def_readwrite("new_position", &OutputParameter<1>::new_position)
        .def_readwrite("new_velocity", &OutputParameter<1>::new_velocity)
        .def_readwrite("new_acceleration", &OutputParameter<1>::new_acceleration)
        .def("__copy__",  [](const OutputParameter<1> &self) {
            return OutputParameter<1>(self);
        });

    py::class_<Quintic<1>>(m, "Quintic")
        .def(py::init<double>(), "delta_time"_a)
        .def_readonly("delta_time", &Quintic<1>::delta_time)
        .def("update", &Quintic<1>::update);

    py::enum_<Quintic<1>::Result>(m, "Result")
        .value("Working", Quintic<1>::Result::Working)
        .value("Finished", Quintic<1>::Result::Finished)
        .value("Error", Quintic<1>::Result::Error)
        .export_values();
}
