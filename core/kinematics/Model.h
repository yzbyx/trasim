//
// Created by yzbyx on 2023/7/9.
//

#ifndef TRASIM_MODEL_H
#define TRASIM_MODEL_H

#include <string>
#include <iostream>
#include <map>

class Vehicle;

class Model {
public:
    Model();
    virtual ~Model();

    virtual void _update_dynamic() = 0;

    virtual std::map<std::string, double> step(int index, ...) = 0;

    void param_update(const std::map<std::string, double>& param);

    Vehicle* vehicle;
    std::string thesis;
    double dt;
    std::map<std::string, double> param_map;

    [[nodiscard]] std::map<std::string, double> get_param_map() const;

    bool has_member(const std::string &name);

    void set_member(const std::string &name, double value);
};

/**
 * KK模型中的速度适配距离计算
 * @return 距离
 */
double cal_G(double k_, double tau_, double a_, double v, double l_v);

#endif //TRASIM_MODEL_H
