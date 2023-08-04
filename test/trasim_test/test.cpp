//
// Created by yzbyx on 2023/8/4.
//
#include "lane_test.h"
#include "road_test.h"
#include "Python.h"

int load_draw() {
    Py_Initialize();
    const char *scriptPath = R"(E:\CProject\trasim\python\run_load_draw.py)";
    FILE* file = fopen(scriptPath, "r");
    if (file) {
        PyRun_SimpleFile(file, scriptPath);
        fclose(file);
    }
    Py_Finalize();
}

int main() {
    lane_test();
//    road_test();

    load_draw();
}