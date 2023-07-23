//
// Created by yzbyx on 2023/7/20.
//

#include "LCModel.h"
#include <random>
#include "../Model.h"
#include "../../Constants.h"
#include "LCModel_KK.h"
//#include "LCModel_ACC.h"


LCModel::LCModel(Vehicle* vehicle_) : Model() {
    name = LCM::NONE;
    vehicle = vehicle_;
    random = & RANDOM::LCM_RND;
    last_lc_time = - std::numeric_limits<double>::infinity();
}

LCModel* get_lc_model(Vehicle* _driver, LCM name, const std::map<std::string, double>& param) {
    if (name == LCM::KK) {
        return new LCModel_KK(_driver, param);
    }
    if (name == LCM::ACC) {
//        return new LCModel_ACC(_driver, param);
    }
    return nullptr; // Return nullptr if name is not provided or not recognized
}
