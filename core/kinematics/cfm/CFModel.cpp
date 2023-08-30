//
// Created by yzbyx on 2023/7/21.
//
#include "CFModel.h"
#include "../../Vehicle.h"
#include "CFM_IDM.h"
#include "CFM_Dummy.h"
#include "../../frame/micro/LaneAbstract.h"


CFModel::CFModel(Vehicle * vehicle_) : Model() {
    name = CFM::NONE;
    random = & RANDOM::CFM_RND;
    vehicle = vehicle_;
}

double CFModel::get_speed_limit() {
    return vehicle->lane->get_speed_limit(vehicle->x, vehicle->type);
}

double CFModel::get_expect_speed() {
    return DEFAULT_EXPECT_SPEED;
}

double CFModel::get_jam_density(double v_length) const {
    return -1;
}

double CFModel::basic_diagram_k_to_q(double dhw, double car_length, double speed_limit) {
    return -1;
}

std::vector<double> CFModel::equilibrium_state(double speed, double dhw, double v_length) {
    return {};
}

void CFModel::get_qm(...) {
}

double CFModel::get_expect_dec() {
    return DEFAULT_EXPECT_DEC;
}

double CFModel::get_expect_acc() {
    return DEFAULT_EXPECT_ACC;
}

CFModel::~CFModel() = default;

CFModel* get_cf_model(Vehicle* _driver, CFM name, const std::map<std::string, double> &param) {
    if (name == CFM::IDM) {
        CFModel *cf = new CFM_IDM(_driver, param);
        return cf;
    } else if (name == CFM::DUMMY) {
        return new CFM_Dummy(_driver, param);
    } else {
        throw;
    }
}