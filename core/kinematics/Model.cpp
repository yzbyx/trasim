//
// Created by yzbyx on 2023/7/9.
//
#include "Model.h"

void Model::param_update(const std::map<std::string, double>& param) {
    for (const auto& pair : param) {
        std::string key = pair.first;
        std::string inner_name = "_" + key;
        if (this->has_member(inner_name)) {
            this->set_member(inner_name, pair.second);
        } else {
            std::cout << "模型无参数" << key << "!" << std::endl;
        }
    }
}

std::map<std::string, double> Model::get_param_map() const {
    return param_map;
}

bool Model::has_member(const std::string& name) {
    return this->param_map.find(name) != this->param_map.end();
}

void Model::set_member(const std::string& name, double value) {
    this->param_map[name] = value;
}

Model::Model() {
    vehicle = nullptr;
    dt = 0.1;
}

Model::~Model() = default;

double cal_G(double k_, double tau_, double a_, double v, double l_v) {
    return std::max(0., k_ * tau_ * v + (1 / a_) * v * (v - l_v));
}
