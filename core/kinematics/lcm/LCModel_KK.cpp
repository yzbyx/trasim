//
// Created by yzbyx on 2023/7/21.
//

#include <cstdarg>
#include "LCModel_KK.h"

LCModel_KK::LCModel_KK(Vehicle* vehicle, const std::map<std::string, double>& l_param)
        : LCModel(vehicle) {
    this->name = LCM::KK;
    this->thesis = "Physics of Automated-Driving Vehicular Traffic";

    this->_a_0 = vehicle->cf_model->get_expect_acc();
    this->_delta_1 = l_param.at("delta_1");
    this->_delta_2 = l_param.at("delta_2");
    this->_gamma_ahead = l_param.at("gamma_ahead");
    this->_gamma_behind = l_param.at("gamma_behind");
    this->_k = l_param.at("k");
    this->_tau = l_param.at("tau");
    this->L_a_ = l_param.at("L_a");
    this->_p_0 = l_param.at("p_0");
    this->_lambda_b = l_param.at("lambda_b");
    this->_delta_vr_1 = l_param.at("delta_vr_1");
    this->xm = NAN;

    this->left_lane = nullptr;
    this->right_lane = nullptr;
}

void LCModel_KK::_update_dynamic() {
    if (!this->vehicle->lane->is_circle) {
        throw std::runtime_error("此换道模型在边界处由于xm");
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

    std::set<SECTION_TYPE> type_ = lane->get_section_type(vehicle->x, vehicle->type);
    if (type_.count(SECTION_TYPE::BASE) > 0) {
        return base_cal();
    } else if (type_.count(SECTION_TYPE::ON_RAMP) > 0 ) {
        return on_ramp_cal();
    } else {
        // You need to define TrasimError and handle the error condition here
        throw std::runtime_error("没有对应的处理函数！");
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
        Vehicle* _f;
        Vehicle* _l;
        std::tie(_f, _l) = this->left_lane->get_relative_car(this->vehicle);
        bool safe_;
        std::tie(safe_, left_d_l) = this->safe_check(_f, _l);
        if (safe_) {
            if (_l != nullptr) {
                if (_l->v >= l_v + this->_delta_1 && this->vehicle->v > l_v) {
                    left_ = true;
                }
            } else {
                left_ = true;
            }
        }
    }

    // 判断是否选择右转
    if (this->right_lane != nullptr) {
        Vehicle* _f;
        Vehicle* _l;
        std::tie(_f, _l) = this->right_lane->get_relative_car(this->vehicle);
        bool safe_;
        std::tie(safe_, right_d_l) = this->safe_check(_f, _l);
        if (safe_) {
            if (_l != nullptr) {
                if (_l->v >= l_v + this->_delta_2 || _l->v >= this->vehicle->v + this->_delta_2) {
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
        std::tie(_f, _l) = this->left_lane->get_relative_car(this->vehicle);
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
                bool condition_1 = (this->vehicle->pos_list.back() < this->xm && this->vehicle->x >= this->xm);
                bool condition_2 = (this->vehicle->pos_list.back() >= this->xm && this->vehicle->x < this->xm);
                if (condition_1 || condition_2) {
                    head_safe = behind_safe = true;
                    x = this->xm;
                }
            }
        }
    }
    this->xm = xm_;

    return {head_safe && behind_safe, d_l, v_hat, x};
}

std::tuple<double, bool, double, double> LCModel_KK::safe_func_on_ramp_common(Vehicle* follower, Vehicle* leader, double v_hat) const {
    double d_l = -leader->get_dist(follower->x) - leader->length;
    double D;
    if (v_hat == NAN) {
        v_hat = std::min(leader->v, follower->v + this->_delta_vr_1);
        D = cal_G(this->_k, this->_tau, this->_a_0, v_hat, leader->v);
    } else {
        D = cal_G(this->_k, this->_tau, this->_a_0, follower->v, v_hat);
    }
    bool safe = (d_l >= std::min(v_hat * this->_tau, D));
    return {D, safe, d_l, v_hat};
}
