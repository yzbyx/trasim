//
// Created by yzbyx on 2023/6/23.
//

#include "Vehicle.h"

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
    cf_acc = cf_model->step(index);
}

void Vehicle::step_lane_change(int index, LaneAbstract *left_lane, LaneAbstract *right_lane) {
    lc_result = lc_model->step(index, left_lane, right_lane);
    lc_res_pre = lc_result;
}


