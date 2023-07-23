//
// Created by yzbyx on 2023/7/9.
//

#include "CFM_Dummy.h"
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

void CFM_Dummy::_update_dynamic() {

}

std::map<std::string, double> CFM_Dummy::step(int index, ...) {
    return {{"a", 0}};
}

CFM_Dummy::CFM_Dummy(Vehicle *vehicle_, const std::map<std::string, double> &f_param)
        : CFModel(vehicle_) {
    name = CFM::DUMMY;
    thesis = "None";
}
