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
    py::class_<Gripper>(m, "Gripper")
        .def(py::init<const std::string&>());
}