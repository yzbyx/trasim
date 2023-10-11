//
// Created by yzbyx on 2023/7/9.
//

#ifndef TRASIM_CFM_IDM_H
#define TRASIM_CFM_IDM_H

#include <string>
#include <map>
#include "CFModel.h"

class Vehicle;

class CFM_IDM : public CFModel {
public:
    CFM_IDM(std::shared_ptr<Vehicle> vehicle_, const std::map<std::string, double>& f_param_);
    ~ CFM_IDM() override;

    double get_expect_dec() override;

    double get_expect_acc() override;

    double get_expect_speed() override;

    double _update_dynamic() override;

    static const std::map<std::string, double> default_f_param;

private:
    double _v0;
    double _s0;
    double _s1;
    double _delta;
    double _t;
    double _omega;
    double _d;

    std::vector<double> equilibrium_state(double speed, double dhw, double v_length) override;

    [[nodiscard]] double get_jam_density(double car_length) const override;

    double step(int index) override;

    double basic_diagram_k_to_q(double dhw, double car_length, double speed_limit) override;
};


#endif //TRASIM_CFM_IDM_H
