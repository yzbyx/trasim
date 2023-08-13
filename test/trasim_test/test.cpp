//
// Created by yzbyx on 2023/8/4.
//
#define PY_SSIZE_T_CLEAN
//#include <Python.h>
#include "lane_test.h"
#include "road_test.h"

//int load_draw() {
//    Py_Initialize();
//    const char *scriptPath = "E:\\CProject\\trasim\\python\\run_load_draw.py";
//
//    PyObject* pModule = nullptr;
//    PyObject* pFunc = nullptr;
//    PyObject* pName = nullptr;
//
//    pModule = PyImport_ImportModule(scriptPath);
//    pFunc = PyObject_GetAttrString(pModule, "run");
//    PyEval_CallObjectWithKeywords(pFunc, nullptr, nullptr);
//
////    PyRun_SimpleString("import sys");
////    PyRun_SimpleString((std::string("execfile('") + scriptPath +  "')").c_str());
//    Py_Finalize();
//    return 0;
//}

int main() {
//    lane_test();
    road_test();

//    load_draw();
}
