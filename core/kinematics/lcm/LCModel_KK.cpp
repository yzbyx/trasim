//
// Created by yzbyx on 2023/7/21.
//

#include <cstdarg>
#include "LCModel_KK.h"
#include "../../frame/micro/LaneAbstract.h"
#include "../cfm/CFModel.h"

const std::map<std::string, double> LCModel_KK::default_f_param = {
        {"delta_2", 5},
        {"gamma_ahead", 1},
        {"gamma_behind", 0.5},
        {"k", 3.5},
        {"tau", 1},
        {"L_a", 150},
        {"p_0", 0.45},
        {"lambda_b", 0.75},
        {"delta_vr_1", 10.}
};

LCModel_KK::LCModel_KK(Vehicle* vehicle, const std::map<std::string, double>& l_param_)
        : LCModel(vehicle) {
    this->name = LCM::KK;
    this->thesis = "Physics of Automated-Driving Vehicular Traffic";

    param_update(default_f_param);
    param_update(l_param_);
    this->_a_0 = vehicle->cf_model->get_expect_acc();
    param_map["delta_1"] = 2 * _a_0 * vehicle->lane->dt;
    this->_delta_1 = param_map["delta_1"];
    this->_delta_2 = param_map["delta_2"];
    this->_gamma_ahead = param_map["gamma_ahead"];
    this->_gamma_behind = param_map["gamma_behind"];
    this->_k = param_map["k"];
    this->_tau = param_map["tau"];
    this->L_a_ = param_map["L_a"];
    this->_p_0 = param_map["p_0"];
    this->_lambda_b = param_map["lambda_b"];
    this->_delta_vr_1 = param_map["delta_vr_1"];

    this->xm = NAN;

    this->left_lane = nullptr;
    this->right_lane = nullptr;
}

void LCModel_KK::_update_dynamic() {
    if (this->vehicle->lane->is_circle) {
        throw std::runtime_error("This lane change model may have errors on the circular boundary(due to 'xm').");
    }
    this->lane = this->vehicle->lane;
    this->dt = this->lane->dt;
}

std::map<std::string, double> LCModel_KK::step(int index, ...) {
    _update_dynamic();
    va_list args; // Declare a va_list to handle variable arguments
    va_start(args, index); // Initialize the va_list
    left_lane = va_arg(args, LaneAbstract *); // Get the left_lane argument from variable arguments
    right_lane = va_arg(args, LaneAbstract *); // Get the right_lane argument from variable arguments
    va_end(args); // Clean up the va_list

    std::vector<SECTION_TYPE> type_ = vehicle->section_type;
    if (std::find(type_.begin(), type_.end(), SECTION_TYPE::BASE) != type_.end()) {
        return base_cal();
    } else if (std::find(type_.begin(), type_.end(), SECTION_TYPE::ON_RAMP) != type_.end()) {
        return on_ramp_cal();
    } else {
        // You need to define TrasimError and handle the error condition here
        throw std::runtime_error("There is no corresponding processing function!");
    }
}

std::map<std::string, double> LCModel_KK::base_cal() {
    if (this->vehicle->leader == nullptr) {
        return {{"lc", 0.}};
    }

    bool left_ = false;
    bool right_ = false;
    double left_d_l = -1;
    double right_d_l = -1;

    double l_v = this->vehicle->leader->v;
    if (this->vehicle->dhw() < this->L_a_) {
        l_v = std::numeric_limits<double>::infinity();
    }

    // 判断是否选择左转
    if (this->left_lane != nullptr) {
        Vehicle* left_f;
        Vehicle* left_l;
        std::tie(left_f, left_l) = this->left_lane->get_relative_car(this->vehicle->x);
        bool safe_;
        std::tie(safe_, left_d_l) = this->safe_check(left_f, left_l);
        if (safe_) {
            if (left_l != nullptr) {
                if (left_l->v >= l_v + this->_delta_1 && this->vehicle->v > l_v) {
                    left_ = true;
                }
            } else {
                left_ = true;
            }
        }
    }

    // 判断是否选择右转
    if (this->right_lane != nullptr) {
        Vehicle* right_f;
        Vehicle* right_l;
        std::tie(right_f, right_l) = this->right_lane->get_relative_car(this->vehicle->x);
        bool safe_;
        std::tie(safe_, right_d_l) = this->safe_check(right_f, right_l);
        if (safe_) {
            if (right_l != nullptr) {
                if (right_l->v >= l_v + this->_delta_2 || right_l->v >= this->vehicle->v + this->_delta_2) {
                    right_ = true;
                }
            } else {
                right_ = true;
            }
        }
    }

    // 概率选择是否换道
    if (left_ || right_) {
        if (RANDOM::DIS12(RANDOM::LCM_RND) < this->_p_0) {
            if (left_ && right_) {
                int direct = left_d_l > right_d_l ? -1 : 1;
                return {{"lc", direct}};
            } else if (left_) {
                return {{"lc", -1}};
            } else {
                return {{"lc", 1}};
            }
        }
    }

    return {{"lc", 0}};
}

std::tuple<bool, double> LCModel_KK::safe_check(Vehicle* _f, Vehicle* _l) {
    double d_l, D_ahead, d_f, D_behind;
    bool head_safe, behind_safe;

    if (_l != nullptr) {
        d_l = -_l->get_dist(this->vehicle->x) - _l->length;
        D_ahead = cal_G(this->_k, this->_tau, this->_a_0, this->vehicle->v, _l->v);
        head_safe = (d_l > std::min(this->_gamma_ahead * this->vehicle->v * this->_tau + _l->length, D_ahead));
    } else {
        head_safe = true;
        d_l = std::numeric_limits<double>::infinity();
    }

    if (_f != nullptr) {
        d_f = _f->get_dist(this->vehicle->x) - this->vehicle->length;
        D_behind = cal_G(this->_k, this->_tau, this->_a_0, _f->v, this->vehicle->v);
        behind_safe = (d_f > std::min(this->_gamma_behind * _f->v * this->_tau + this->vehicle->length, D_behind));
    } else {
        behind_safe = true;
    }

    return {head_safe && behind_safe, d_l};
}

std::map<std::string, double> LCModel_KK::on_ramp_cal() {
    // 限制仅向左换道
    if (this->left_lane != nullptr) {
        Vehicle* _f;
        Vehicle* _l;
        std::tie(_f, _l) = this->left_lane->get_relative_car(this->vehicle->x);
        bool safe_;
        double left_d_l, v_hat, x;
        std::tie(safe_, left_d_l, v_hat, x) = this->safe_check_on_ramp(_f, _l);
        if (safe_) {
            return {{"lc", -1}, {"x", x}, {"v", v_hat}};
        }
    }

    return {{"lc", 0}};
}

std::tuple<bool, double, double, double> LCModel_KK::safe_check_on_ramp(Vehicle* _f, Vehicle* _l) {
    bool head_safe = false;
    double D_ahead, d_l, v_hat;
    double x = this->vehicle->x;

    if (_l != nullptr) {
        std::tie(D_ahead, head_safe, d_l, v_hat) = this->safe_func_on_ramp_common(this->vehicle, _l, NAN);
    } else {
        v_hat = this->vehicle->v + this->_delta_vr_1;
        head_safe = true;
        d_l = std::numeric_limits<double>::infinity();
    }

    double D_behind;
    bool behind_safe = false;
    if (_f != nullptr) {
        std::tie(D_behind, behind_safe, std::ignore, std::ignore) = this->safe_func_on_ramp_common(_f, this->vehicle, v_hat);
    } else {
        behind_safe = true;
    }

    double xm_ = std::numeric_limits<double>::quiet_NaN();
    if (_l != nullptr && _f != nullptr) {
        xm_ = _f->x + _f->dhw() / 2;
    }
    if (!head_safe || !behind_safe) {
        if (_l != nullptr && _f != nullptr) {
            if (_f->gap() > this->_lambda_b * _f->v + this->vehicle->length) {
                bool condition_1 = (this->vehicle->pos_list.back() < this->xm && this->vehicle->x >= xm_);
                bool condition_2 = (this->vehicle->pos_list.back() >= this->xm && this->vehicle->x < xm_);
                if (condition_1 || condition_2) {
                    head_safe = behind_safe = true;
                    x = xm_;
                }
            }
        }
    }
    this->xm = xm_;

    return {head_safe && behind_safe, d_l, v_hat, x};
}

/**
 *
 * @param follower
 * @param leader
 * @param v_hat 是否为ego的后车和ego, 若是则需要传ego的v_hat
 * @return
 */
std::tuple<double, bool, double, double> LCModel_KK::safe_func_on_ramp_common(Vehicle* follower, Vehicle* leader, double v_hat) const {
    double d_l = -leader->get_dist(follower->x) - leader->length;
    double D;
    if (std::isnan(v_hat)) {
        v_hat = std::min(leader->v, follower->v + this->_delta_vr_1);
        D = cal_G(this->_k, this->_tau, this->_a_0, v_hat, leader->v);
    } else {
        D = cal_G(this->_k, this->_tau, this->_a_0, follower->v, v_hat);
    }
    bool safe = (d_l >= std::min(v_hat * this->_tau, D));
    return {D, safe, d_l, v_hat};
}
