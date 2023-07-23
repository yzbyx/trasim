//
// Created by yzbyx on 2023/6/23.
//

#include "Vehicle.h"
#include "frame/micro/LaneAbstract.h"


Vehicle::Vehicle(LaneAbstract *lane_, VType type_, int id_, double length_) : Obstacle(type_) {
    ID = id_;

    length = length_;
    lane = lane_;

    leader = follower = nullptr;

    cf_model = nullptr;
    lc_model = nullptr;

    cf_acc = 0;

    lc_result = {{"lc", 0}, {"a", 0}, {"v", 0}, {"x", 0}};
    lc_res_pre = lc_result;

    ttc_star = 1.3;
    is_run_out = false;
}

double Vehicle::last_step_lc_status() {
    return lc_res_pre["lc"];
}

void Vehicle::set_cf_model(CFM cf_name, const std::map<std::string, double> &cf_param) {
    cf_model = get_cf_model(this, cf_name, cf_param);
}

void Vehicle::set_lc_model(LCM lc_name, const std::map<std::string, double> &lc_param) {
    lc_model = get_lc_model(this, lc_name, lc_param);
}

void Vehicle::step(int index) {
    cf_acc = cf_model->step(index)["a"];
}

void Vehicle::step_lane_change(int index, LaneAbstract *left_lane, LaneAbstract *right_lane) {
    lc_result = lc_model->step(index, left_lane, right_lane);
    lc_res_pre = lc_result;
}

double Vehicle::get_dist(double pos) {
    if (lane->is_circle) {
        double dist_head, dist_after, dist;
        if (pos > x) {
            dist_head = pos - x;
            dist_after = lane->lane_length - pos + x;
            dist = dist_head < dist_after ? dist_head : (-dist_after);
        } else {
            dist_head = pos + lane->lane_length - x;
            dist_after = x - pos;
            dist = dist_head < dist_after ? dist_head : (-dist_after);
        }
        return dist;
    } else {
        return pos - x;
    }
}

bool Vehicle::is_first() const {
    return leader == nullptr || leader->type == VType::OBSTACLE;
}

std::map<C_Info, std::vector<double>> Vehicle::get_data_list(C_Info info) {
    std::map<C_Info, std::vector<double>> result;

    if (info == C_Info::lane_add_num) {
        result[info] = lane_id_list;
    } else if (info == C_Info::id) {
        std::vector<double> id_list(pos_list.size(), ID);
        result[info] = id_list;
    } else if (info == C_Info::car_type) {
        std::vector<double> type_list(pos_list.size(), static_cast<int>(type));
        result[info] = type_list;
    } else if (info == C_Info::a) {
        result[info] = acc_list;
    } else if (info == C_Info::v) {
        result[info] = speed_list;
    } else if (info == C_Info::x) {
        result[info] = pos_list;
    } else if (info == C_Info::dv) {
        result[info] = dv_list;
    } else if (info == C_Info::gap) {
        result[info] = gap_list;
    } else if (info == C_Info::dhw) {
        result[info] = dhw_list;
    } else if (info == C_Info::thw) {
        result[info] = thw_list;
    } else if (info == C_Info::time) {
        result[info] = time_list;
    } else if (info == C_Info::step) {
        result[info] = step_list;
    } else if (info == C_Info::cf_id) {
        std::vector<double> cf_id_list(pos_list.size(), static_cast<int>(cf_model->name));
        result[info] = cf_id_list;
    } else if (info == C_Info::lc_id) {
        std::vector<double> lc_id_list(pos_list.size(), static_cast<int>(lc_model->name));
        result[info] = lc_id_list;
    } else if (info == C_Info::safe_ttc) {
        result[info] = ttc_list;
    } else if (info == C_Info::safe_tit) {
        result[info] = tit_list;
    } else if (info == C_Info::safe_tet) {
        result[info] = tet_list;
    } else if (info == C_Info::safe_picud) {
        result[info] = picud_list;
    } else if (info == C_Info::safe_picud_KK) {
        result[info] = picud_KK_list;
    } else {
        throw std::runtime_error("C_Info未创建！");
    }

    return result;
}

void Vehicle::record() {
    for (const auto& info : lane->data_container->save_info) {
        switch (info) {
            case C_Info::lane_add_num:
                lane_id_list.push_back(lane->add_num);
                break;
            case C_Info::a:
                acc_list.push_back(a);
                break;
            case C_Info::v:
                speed_list.push_back(v);
                break;
            case C_Info::x:
                pos_list.push_back(x);
                break;
            case C_Info::dv:
                dv_list.push_back(dv());
                break;
            case C_Info::gap: {
                gap_list.push_back(gap());
                break;
            }
            case C_Info::dhw:
                dhw_list.push_back(dhw());
                break;
            case C_Info::thw:
                thw_list.push_back(thw());
                break;
            case C_Info::time:
                time_list.push_back(lane->time_);
                break;
            case C_Info::step:
                step_list.push_back(lane->step_);
                break;
            case C_Info::safe_ttc:
                ttc_list.push_back(ttc());
                break;
            case C_Info::safe_tit:
                tit_list.push_back(tit());
                break;
            case C_Info::safe_tet:
                tet_list.push_back(tet());
                break;
            case C_Info::safe_picud:
                picud_list.push_back(picud());
                break;
            case C_Info::safe_picud_KK:
                picud_KK_list.push_back(picud_KK());
                break;
            default:
                throw std::runtime_error("C_Info未创建！");
        }
    }
}

double Vehicle::gap() {
    if (leader != nullptr) {
        double dhw = this->dhw();
        double gap = dhw - leader->length;
        return gap;
    } else {
        return std::numeric_limits<double>::quiet_NaN();
    }
}

double Vehicle::dv() {
    if (leader != nullptr) {
        return leader->v - v;
    } else {
        return std::numeric_limits<double>::quiet_NaN();
    }
}

double Vehicle::dhw() {
    if (leader != nullptr) {
        double dhw = leader->x - x;
        if (dhw < 0) {
            if (lane->is_circle && lane->car_list.back()->ID == ID) {
                dhw += lane->lane_length;
            } else {
                throw std::runtime_error("车头间距小于0！\n");
            }
        }
        return dhw;
    } else {
        return std::numeric_limits<double>::quiet_NaN();
    }
}

double Vehicle::thw() {
    if (leader != nullptr) {
        if (dv() != 0) {
            return dhw() / (-dv());
        }
    }
    return std::numeric_limits<double>::quiet_NaN();
}

double Vehicle::ttc() {
    if (leader != nullptr) {
        if (dv() != 0) {
            return gap() / (-dv());
        }
    }
    return std::numeric_limits<double>::quiet_NaN();
}

double Vehicle::tit() {
    double ttc = this->ttc();
    if (0 <= ttc && ttc <= ttc_star) {
        return ttc_star - ttc;
    }
    return 0;
}

double Vehicle::tet() {
    double ttc = this->ttc();
    if (0 <= ttc && ttc <= ttc_star) {
        return 1;
    }
    return 0;
}

double Vehicle::picud() {
    if (leader != nullptr) {
        double l_dec = leader->cf_model->get_expect_dec();
        double l_v = leader->v;
        double l_x = (leader->x > x) ? leader->x : (leader->x + lane->lane_length);
        double l_length = leader->length;
        double dec = cf_model->get_expect_dec();
        double xd_l = (l_v * l_v) / (2 * l_dec);
        double xd = (v * v) / (2 * dec);
        return (l_x + xd_l) - (x + v * lane->dt + xd) - l_length;
    } else {
        return std::numeric_limits<double>::quiet_NaN();
    }
}

double Vehicle::picud_KK() {
    if (leader != nullptr) {
        double l_v = leader->v;
        double l_x = (leader->x > x) ? leader->x : (leader->x + lane->lane_length);
        double l_length = leader->length;
        double dec = cf_model->get_expect_dec();
        double tau = lane->dt;

        int alpha = static_cast<int>(l_v / (dec * tau));  // 使用当前车的最大期望减速度
        double beta = l_v / (dec * tau) - alpha;
        double xd_l = dec * tau * tau * (alpha * beta + 0.5 * alpha * (alpha - 1));

        alpha = static_cast<int>(v / (dec * tau));
        beta = v / (dec * tau) - alpha;
        double xd = dec * tau * tau * (alpha * beta + 0.5 * alpha * (alpha - 1));

        return (l_x + xd_l) - (x + v * tau + xd) - l_length;
    } else {
        return std::numeric_limits<double>::quiet_NaN();
    }
}

bool Vehicle::has_data() const {
    return !pos_list.empty();
}

void Vehicle::set_car_param(std::map<std::string, double> & param) {
    color = static_cast<Color>(param["color"]);
    width = param["width"];
}
