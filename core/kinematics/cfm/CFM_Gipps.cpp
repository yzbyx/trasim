//
// Created by yzbyx on 2023/9/20.
//

#include "CFM_Gipps.h"

#include <utility>
#include "../../Vehicle.h"
#include "../../frame/micro/LaneAbstract.h"

const std::map<std::string, double> CFM_Gipps::default_f_param = {
        {"a", 2},
        {"b", -3},
        {"v0", 20},
        {"tau", 0.7},
        {"s", 6.5},
        {"b_hat", -2.5}
};

CFM_Gipps::CFM_Gipps(std::shared_ptr<Vehicle> vehicle_, const std::map<std::string, double> &f_param_)
        : CFModel(std::move(vehicle_)) {
    param_update(default_f_param);
    param_update(f_param_);

    _a = param_map["a"];
    _b = param_map["b"];
    _v0 = param_map["v0"];
    _tau = param_map["tau"];
    _s = param_map["s"];
    _b_hat = param_map["b_hat"];

    pre_tau_v = -1;
    pre_tau_dhw = -1;
    pre_tau_lv = -1;

    name = CFM::Gipps;
    thesis = "A behavioural car-following model for computer simulation (1981)";
}

double CFM_Gipps::_update_dynamic() {
    if (_tau == vehicle->lane->dt) {
        pre_tau_v = vehicle->v;
        pre_tau_lv = vehicle->leader->v;
        pre_tau_dhw = vehicle->dhw();
    } else {
#ifdef DynamicTau
        int offset = int(std::round(vehicle->lane->dt / _tau));
        ...
#else
        throw std::runtime_error("tau and dt should be equal!");
#endif
    }
}

double CFM_Gipps::step(int index) {
    if (vehicle->leader == nullptr) {
        return get_expect_acc();
    }
    _update_dynamic();

    return calculate(_a, _b, _v0, _tau, _s, _b_hat,
                     pre_tau_v, pre_tau_dhw, pre_tau_lv);
}

double CFM_Gipps::get_expect_dec() {
    return - _b;
}

double CFM_Gipps::get_expect_acc() {
    return _a;
}

double CFM_Gipps::get_expect_speed() {
    return std::min(get_speed_limit(), _v0);
}

double
CFM_Gipps::calculate(double a, double b, double v0, double tau, double s, double b_hat, double speed, double dhw,
                     double leader_v) {
    // 包络线公式限制
    double v_max1 = speed + 2.5 * a * tau * (1 - speed / v0) * std::pow(0.025 + speed / v0, 0.5);
    // 安全驾驶限制，注意此处的s为当前车与前车的期望车头间距
    double v_max2 = b * tau + std::sqrt((b * b) * (tau * tau) - b * (2 * (dhw - s) - speed * tau - (leader_v * leader_v) / b_hat));
    // 选取最小的速度限制作为下一时刻t+tau的速度
    double v_tau = std::min(v_max1, v_max2);
    // 计算加速度
    return (v_tau - speed) / tau;
}

CFM_Gipps::~CFM_Gipps() = default;