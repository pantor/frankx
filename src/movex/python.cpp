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
#include <movex/path/time_parametrization.hpp>
#include <movex/path/trajectory.hpp>

#ifdef WITH_REFLEXXES
    #include <movex/otg/reflexxes.hpp>
#endif


namespace py = pybind11;
using namespace pybind11::literals; // to bring in the `_a` literal
using namespace movex;


PYBIND11_MODULE(_movex, m) {
    m.doc() = "Robot Motion Library with Focus on Online Trajectory Generation";

    constexpr size_t DOFs {2};

    py::class_<Affine>(m, "Affine")
        .def(py::init<double, double, double, double, double, double>(), "x"_a=0.0, "y"_a=0.0, "z"_a=0.0, "a"_a=0.0, "b"_a=0.0, "c"_a=0.0)
        .def(py::init<Vector6d>())
        .def(py::init<Vector7d>())
        .def(py::init<const std::array<double, 16>&>(), "data"_a)
        .def(py::init<const Affine &>(), "affine"_a) // Copy constructor
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
        .def("slerp", &Affine::slerp, "affine"_a, "t"_a)
        .def("__repr__", &Affine::toString);

    py::class_<InputParameter<DOFs>>(m, "InputParameter")
        .def(py::init<>())
        .def_readonly_static("degrees_of_freedom", &InputParameter<DOFs>::degrees_of_freedom)
        .def_readwrite("current_position", &InputParameter<DOFs>::current_position)
        .def_readwrite("current_velocity", &InputParameter<DOFs>::current_velocity)
        .def_readwrite("current_acceleration", &InputParameter<DOFs>::current_acceleration)
        .def_readwrite("target_position", &InputParameter<DOFs>::target_position)
        .def_readwrite("target_velocity", &InputParameter<DOFs>::target_velocity)
        .def_readwrite("target_acceleration", &InputParameter<DOFs>::target_acceleration)
        .def_readwrite("max_velocity", &InputParameter<DOFs>::max_velocity)
        .def_readwrite("max_acceleration", &InputParameter<DOFs>::max_acceleration)
        .def_readwrite("max_jerk", &InputParameter<DOFs>::max_jerk)
        .def_readwrite("minimum_duration", &InputParameter<DOFs>::minimum_duration);

    py::class_<OutputParameter<DOFs>>(m, "OutputParameter")
        .def(py::init<>())
        .def_readwrite("new_position", &OutputParameter<DOFs>::new_position)
        .def_readwrite("new_velocity", &OutputParameter<DOFs>::new_velocity)
        .def_readwrite("new_acceleration", &OutputParameter<DOFs>::new_acceleration)
        .def_readwrite("duration", &OutputParameter<DOFs>::duration)
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
        .def_readonly("last_calculation_duration", &Ruckig<DOFs>::last_calculation_duration)
        .def("update", &Ruckig<DOFs>::update)
        .def("at_time", &Ruckig<DOFs>::atTime);

#ifdef WITH_REFLEXXES
    py::class_<Reflexxes<DOFs>>(m, "Reflexxes")
        .def(py::init<double>(), "delta_time"_a)
        .def_readonly("delta_time", &Reflexxes<DOFs>::delta_time)
        .def("update", &Reflexxes<DOFs>::update)
        .def("at_time", &Reflexxes<DOFs>::atTime);
#endif

    py::class_<Path>(m, "Path")
        .def(py::init<const std::vector<Waypoint>&>(), "waypoints"_a)
        .def(py::init<const std::vector<Affine>&, double>(), "waypoints"_a, "blend_max_distance"_a = 0.0)
        .def_readonly_static("degrees_of_freedom", &Path::degrees_of_freedom)
        .def_property_readonly("length", &Path::get_length)
        .def("q", (Vector7d (Path::*)(double) const)&Path::q, "s"_a)
        .def("q", (Vector7d (Path::*)(double, const Affine&) const)&Path::q, "s"_a, "frame"_a)
        .def("pdq", &Path::pdq, "s"_a)
        .def("pddq", &Path::pddq, "s"_a)
        .def("pdddq", &Path::pdddq, "s"_a)
        .def("dq", &Path::dq, "s"_a, "ds"_a)
        .def("ddq", &Path::ddq, "s"_a, "ds"_a, "dds"_a)
        .def("dddq", &Path::dddq, "s"_a, "ds"_a, "dds"_a, "ddds"_a)
        .def("max_pddq", &Path::max_pddq)
        .def("max_pdddq", &Path::max_pdddq);

    py::class_<Trajectory::State>(m, "TrajectoryState")
        .def_readwrite("t", &Trajectory::State::t)
        .def_readwrite("s", &Trajectory::State::s)
        .def_readwrite("ds", &Trajectory::State::ds)
        .def_readwrite("dds", &Trajectory::State::dds)
        .def_readwrite("ddds", &Trajectory::State::ddds);

    py::class_<Trajectory>(m, "Trajectory")
        .def_readwrite("path", &Trajectory::path)
        .def_readwrite("states", &Trajectory::states);

    py::class_<TimeParametrization>(m, "TimeParametrization")
        .def(py::init<double>(), "delta_time"_a)
        .def("parametrize", &TimeParametrization::parametrize, "path"_a, "max_velocity"_a, "max_accleration"_a, "max_jerk"_a);
}
