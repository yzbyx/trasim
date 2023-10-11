//
// Created by yzbyx on 2023/7/20.
//

#include "LCModel.h"
#include <random>
#include <memory>
#include <utility>
#include "LCModel_KK.h"


LCModel::LCModel(std::shared_ptr<Vehicle> vehicle_) : Model() {
    name = LCM::NONE;
    vehicle = std::move(vehicle_);
    random = & RANDOM::LCM_RND;
    last_lc_time = - std::numeric_limits<double>::infinity();
}

std::shared_ptr<LCModel> get_lc_model(const std::shared_ptr<Vehicle>& _driver,
                                      LCM name,
                                      const std::map<std::string, double>& param) {
    if (name == LCM::KK) {
        auto lc = std::make_shared<LCModel_KK>(_driver, param);
        return lc;
    }
    else if (name == LCM::ACC) {
//        return new LCModel_ACC(_driver, param);
    }
    return nullptr; // Return nullptr if name is not provided or not recognized
}
