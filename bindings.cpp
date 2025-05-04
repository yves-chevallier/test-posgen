#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "scurve.hpp"

namespace py = pybind11;

PYBIND11_MODULE(scurvecpp , m) {
    m.doc() = "S-Curve motion planner bindings using pybind11";

    // Binding de ContextScurvePlanner
    py::class_<ContextScurvePlanner>(m, "ContextScurvePlanner")
        .def(py::init<>())
        .def_readwrite("velocityLimit", &ContextScurvePlanner::velocityLimit)
        .def_readwrite("acceleration", &ContextScurvePlanner::acceleration)
        .def_readwrite("deceleration", &ContextScurvePlanner::deceleration)
        .def_readwrite("jerk_time", &ContextScurvePlanner::jerk_time);

    // Binding de SCurvePlanner
    py::class_<SCurvePlanner>(m, "SCurvePlanner")
        .def(py::init<double>(), py::arg("dt"))
        .def("set_context", &SCurvePlanner::set_context, py::arg("ctx"))
        .def("apply_context", &SCurvePlanner::apply_context)
        .def("set_distance", &SCurvePlanner::set_distance, py::arg("value"))
        .def("stop", &SCurvePlanner::stop, py::arg("deceleration") = -1.0)
        .def("start", &SCurvePlanner::start)
        .def("is_finished", &SCurvePlanner::isFinished)
        .def("step", [](SCurvePlanner& self) {
            double v = 0.0, a = 0.0;
            bool done = self.step(v, a);
            return py::make_tuple(v, a, done);
        }, "Returns (velocity, acceleration, finished)");

    // Binding de Distretizer
    py::class_<Distretizer>(m, "Distretizer")
        .def(py::init<>())
        .def("step", &Distretizer::step, py::arg("sample"));
}
