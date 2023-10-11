//
// Created by yzbyx on 2023/7/9.
//

#include "CFM_IDM.h"

#include <utility>
#include "../../Vehicle.h"

const std::map<std::string, double> CFM_IDM::default_f_param = {
        {"v0",    33.3},
        {"s0",    2},
        {"s1",    0},
        {"delta", 4},
        {"T",     1.6},
        {"omega", 0.73},
        {"d",     1.67}
};

CFM_IDM::CFM_IDM(std::shared_ptr<Vehicle> vehicle_, const std::map<std::string, double> &f_param_) : CFModel(std::move(vehicle_)) {
    param_update(default_f_param);
    param_update(f_param_);

    /**
     * 期望速度
     */
    _v0 = param_map["v0"];
    /**
     * 静止安全间距
     */
    _s0 = param_map["s0"];
    /**
     * 与速度相关的安全距离参数
     */
    _s1 = param_map["s1"];
    /**
     * 加速度指数
     */
    _delta = param_map["delta"];
    /**
     * 安全车头时距
     */
    _t = param_map["T"];
    /**
     * 舒适加速度
     */
    _omega = param_map["omega"];
    /**
     * 舒适减速度
     */
    _d = param_map["d"];

    name = CFM::IDM;
    thesis = "Congested traffic states in empirical observations and microscopic simulations (2000)";
}

double CFM_IDM::get_expect_dec() {
    return -this->_d;
}

double CFM_IDM::get_expect_acc() {
    return this->_omega * (1 - std::pow(this->vehicle->v / this->_v0, _delta));
}

double CFM_IDM::_update_dynamic() {

}

/**
 * 计算下一时间步的加速度
 * @param index
 * @return
 */
double CFM_IDM::step(int index) {
    if (vehicle->leader == nullptr) {
        return get_expect_acc();
    }
    _update_dynamic();

    double sStar = _s0 + _s1 * sqrt(vehicle->v / _v0) + _t * vehicle->v +
                   vehicle->v * (vehicle->v - vehicle->leader->v) / (2 * sqrt(_omega * _d));
    double finalAcc = _omega * (1 - pow(vehicle->v / _v0, _delta) - pow(sStar / vehicle->gap(), 2));

    return finalAcc;
}

/**
 * 通过平衡态速度计算三参数
 */
std::vector<double> CFM_IDM::equilibrium_state(double speed, double dhw, double v_length) {
    double sStar = _s0 + _s1 * sqrt(speed / _v0) + _t * speed;
    dhw = sStar / sqrt(1 - pow(speed / _v0, _delta)) + v_length;
    double k = 1 / dhw;
    double v = speed;
    double q = k * v;
    std::vector<double> result = {k, q, v};
    return result;
}

/**
 * 基本图 K-Q 线
 */
double CFM_IDM::basic_diagram_k_to_q(double dhw, double car_length, double speed_limit) {
    double v0 = (speed_limit > 0) ? speed_limit : _v0;
    double v;
    // 使用二分法求解 v
    double left = 0, right = v0;
    double tol = 1e-6;
    int max_iter = 100;
    while (max_iter--) {
        v = (left + right) / 2;
        double expr = _omega * (1 - pow(v / v0, _delta) -
                                pow((_s0 + _s1 * sqrt(v / v0) + _t * v) / (dhw - car_length), 2));
        if (expr > 0) {
            right = v;
        } else {
            left = v;
        }
        if (fabs(expr) < tol) {
            break;
        }
    }
    return v * 3.6; // 将速度转换为 km/h
}

/**
 * 获取拥堵密度
 * @param car_length
 * @return
 */
double CFM_IDM::get_jam_density(double car_length) const {
    return 1 / (_s0 + car_length);
}

double CFM_IDM::get_expect_speed() {
    return std::min(get_speed_limit(), _v0);
}

CFM_IDM::~CFM_IDM() = default;
