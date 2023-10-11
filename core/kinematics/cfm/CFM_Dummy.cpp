//
// Created by yzbyx on 2023/7/9.
//

#include "CFM_Dummy.h"

#include <utility>
#include "../../Vehicle.h"

double CFM_Dummy::get_expect_dec() {
    return DEFAULT_EXPECT_DEC;
}

double CFM_Dummy::get_expect_acc() {
    return DEFAULT_EXPECT_ACC;
}

double CFM_Dummy::get_expect_speed() {
    return 0.;
}

double CFM_Dummy::_update_dynamic() {

}

double CFM_Dummy::step(int index) {
    return 0;
}

CFM_Dummy::CFM_Dummy(std::shared_ptr<Vehicle> vehicle_, const std::map<std::string, double> &f_param)
        : CFModel(std::move(vehicle_)) {
    name = CFM::DUMMY;
    thesis = "None";
}
