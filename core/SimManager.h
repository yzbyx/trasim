//
// Created by yzbyx on 2023/9/19.
//

#ifndef TRASIM_SIMMANAGER_H
#define TRASIM_SIMMANAGER_H

#include <vector>

/**
 * SimManager类用于管理整个仿真过程
 * 多仿真场景并行计算
 */
class SimManager {
public:
    class SimScene {

    };

    std::vector<SimScene> scenes{};

    void add_scene();


};


#endif //TRASIM_SIMMANAGER_H
