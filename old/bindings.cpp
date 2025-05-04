#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "positionGenerator.hpp"

namespace py = pybind11;

PYBIND11_MODULE(positiongen, m) {
    py::class_<PositionGeneratorContext>(m, "PositionGeneratorContext")
        .def(py::init<>())
        .def_readwrite("velocity", &PositionGeneratorContext::velocity)
        .def_readwrite("acceleration", &PositionGeneratorContext::acceleration)
        .def_readwrite("deceleration", &PositionGeneratorContext::deceleration);

    py::class_<PositionGenerator>(m, "PositionGenerator")
        .def(py::init<>())
        .def("init", &PositionGenerator::init)
        .def("move", static_cast<void (PositionGenerator::*)(int64_t, float, float, float)>(&PositionGenerator::move))
        .def("update", &PositionGenerator::update)
        .def("target_reached", &PositionGenerator::targetReached);
}
