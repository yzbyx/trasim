//
// Created by yzbyx on 2023/7/9.
//

#ifndef TRASIM_CFM_DUMMY_H
#define TRASIM_CFM_DUMMY_H

#include "CFModel.h"
#include "../../Vehicle.h"
#include <iostream>

class CFM_Dummy : public CFModel {
public:
    CFM_Dummy(Vehicle *vehicle_, const std::map<std::string, double> &f_param);

    double get_expect_dec() override;

    double get_expect_acc() override;

    double get_expect_speed() override;

    void _update_dynamic() override;

    std::map<std::string, double> step(int index, ...) override;
};

#endif //TRASIM_CFM_DUMMY_H
