#include <array>
#include <string>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>

#include <movex/otg/quintic.hpp>
#include <movex/otg/ruckig.hpp>
#include <movex/otg/smoothie.hpp>
#include <movex/path/path.hpp>

#ifdef WITH_REFLEXXES
    #include <movex/otg/reflexxes.hpp>
#endif


namespace py = pybind11;
using namespace pybind11::literals; // to bring in the `_a` literal
using namespace movex;


PYBIND11_MODULE(movex, m) {
    m.doc() = "Robot Motion Library with Focus on Online Trajectory Generation";

    constexpr size_t DOFs {1};

    py::class_<InputParameter<DOFs>>(m, "InputParameter")
        .def(py::init<>())
        .def_readwrite("current_position", &InputParameter<DOFs>::current_position)
        .def_readwrite("current_velocity", &InputParameter<DOFs>::current_velocity)
        .def_readwrite("current_acceleration", &InputParameter<DOFs>::current_acceleration)
        .def_readwrite("target_position", &InputParameter<DOFs>::target_position)
        .def_readwrite("target_velocity", &InputParameter<DOFs>::target_velocity)
        .def_readwrite("target_acceleration", &InputParameter<DOFs>::target_acceleration)
        .def_readwrite("max_velocity", &InputParameter<DOFs>::max_velocity)
        .def_readwrite("max_acceleration", &InputParameter<DOFs>::max_acceleration)
        .def_readwrite("max_jerk", &InputParameter<DOFs>::max_jerk)
        .def_readwrite("minimum_duration", &InputParameter<DOFs>::minimum_duration)
        .def_readonly_static("degrees_of_freedom", &InputParameter<DOFs>::degrees_of_freedom);


    py::class_<OutputParameter<DOFs>>(m, "OutputParameter")
        .def(py::init<>())
        .def_readwrite("new_position", &OutputParameter<DOFs>::new_position)
        .def_readwrite("new_velocity", &OutputParameter<DOFs>::new_velocity)
        .def_readwrite("new_acceleration", &OutputParameter<DOFs>::new_acceleration)
        .def("__copy__",  [](const OutputParameter<DOFs> &self) {
            return OutputParameter<DOFs>(self);
        });

    py::enum_<Result>(m, "Result")
        .value("Working", Result::Working)
        .value("Finished", Result::Finished)
        .value("Error", Result::Error)
        .export_values();

    py::class_<Quintic<DOFs>>(m, "Quintic")
        .def(py::init<double>(), "delta_time"_a)
        .def_readonly("delta_time", &Quintic<DOFs>::delta_time)
        .def("update", &Quintic<DOFs>::update);

    py::class_<Smoothie<DOFs>>(m, "Smoothie")
        .def(py::init<double>(), "delta_time"_a)
        .def_readonly("delta_time", &Smoothie<DOFs>::delta_time)
        .def("update", &Smoothie<DOFs>::update);

    py::class_<Ruckig<DOFs>>(m, "Ruckig")
        .def(py::init<double>(), "delta_time"_a)
        .def_readonly("delta_time", &Ruckig<DOFs>::delta_time)
        .def("update", &Ruckig<DOFs>::update);

#ifdef WITH_REFLEXXES
    py::class_<Reflexxes<DOFs>>(m, "Reflexxes")
        .def(py::init<double>(), "delta_time"_a)
        .def_readonly("delta_time", &Reflexxes<DOFs>::delta_time)
        .def("update", &Reflexxes<DOFs>::update);
#endif

    py::class_<Segment>(m, "Segment");

    py::class_<LineSegment, Segment>(m, "LineSegment")
        .def_property_readonly("length", &LineSegment::get_length)
        .def("q", &LineSegment::q, "s"_a)
        .def("pdq", &LineSegment::pdq, "s"_a)
        .def("pddq", &LineSegment::pddq, "s"_a)
        .def("pdddq", &LineSegment::pdddq, "s"_a);

    py::class_<QuinticSegment, Segment>(m, "QuinticSegment")
        .def_property_readonly("length", &QuinticSegment::get_length)
        .def("q", &QuinticSegment::q, "s"_a)
        .def("pdq", &QuinticSegment::pdq, "s"_a)
        .def("pddq", &QuinticSegment::pddq, "s"_a)
        .def("pdddq", &QuinticSegment::pdddq, "s"_a);

    py::class_<PathPoint>(m, "PathPoint");

    py::class_<Path>(m, "Path")
        .def(py::init<const std::vector<PathPoint>&>(), "waypoints"_a)
        .def_readonly_static("degrees_of_freedom", &Path::degrees_of_freedom)
        .def_static("Linear", &Path::Linear, "waypoints"_a, "blend_max_distance"_a = 0.0)
        .def_property_readonly("length", &Path::get_length)
        .def("q", &Path::q, "s"_a)
        .def("pdq", &Path::pdq, "s"_a)
        .def("pddq", &Path::pddq, "s"_a)
        .def("pdddq", &Path::pdddq, "s"_a);
}
