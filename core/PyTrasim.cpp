//
// Created by yzbyx on 2023/9/12.
//
#include <pybind11/pybind11.h>
#include "frame/micro/Road.h"

namespace py = pybind11;

class Road;

PYBIND11_MODULE(trasim, m) {
    py::enum_<C_Info>(m, "C_Info")
            .value("lane_id", C_Info::lane_add_num)
            .value("id", C_Info::id)
            .value("step", C_Info::step)
            .value("time", C_Info::time)
            .value("a", C_Info::a)
            .value("v", C_Info::v)
            .value("x", C_Info::x)
            .value("dhw", C_Info::dhw)
            .value("thw", C_Info::thw)
            .value("gap", C_Info::gap)
            .value("dv", C_Info::dv)
            .value("cf_id", C_Info::cf_id)
            .value("lc_id", C_Info::lc_id)
            .value("car_type", C_Info::car_type)
            .value("safe_ttc", C_Info::safe_ttc)
            .value("safe_tet", C_Info::safe_tet)
            .value("safe_tit", C_Info::safe_tit)
            .value("safe_picud", C_Info::safe_picud)
            .value("safe_picud", C_Info::safe_picud_KK)
            .export_values();

    py::class_<Road>(m, "Road")
            .def(py::init<double, double>(), py::arg("length") = 1000, py::arg("id") = 0)
            .def("cf_take_over", &Road::cf_take_over)
            .def("get_road_total_data", &Road::get_road_total_data)
            .def("add_lanes", &Road::add_lanes, py::arg("lane_num"), py::arg("is_circle"), py::arg("real_index"))
            .def("get_car_info", &Road::get_car_info, py::arg("car_id"), py::arg("info") = C_Info::x, py::arg("lane_add_num") = -1);

}
