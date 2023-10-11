//
// Created by yzbyx on 2023/9/20.
//

#ifndef TRASIM_CFM_GIPPS_H
#define TRASIM_CFM_GIPPS_H

#include <string>
#include <map>
#include "CFModel.h"

class Vehicle;
class LaneAbstract;

class CFM_Gipps : public CFModel{
public:
    CFM_Gipps(std::shared_ptr<Vehicle> vehicle_, const std::map<std::string, double> &f_param_);
    ~ CFM_Gipps() override;

    double get_expect_dec() override;
    double get_expect_acc() override;
    double get_expect_speed() override;

    double step(int index) override;

    static double calculate(double a, double b, double v0, double tau, double s, double b_hat, double speed,
                            double dhw, double leader_v);

    static const std::map<std::string, double> default_f_param;

private:
    double _a;
    double _b;
    double _v0;
    double _tau;
    double _s;
    double _b_hat;

    double pre_tau_v;
    double pre_tau_dhw;
    double pre_tau_lv;

    std::vector<double> temp_v;
    std::vector<double> temp_dhw;
    std::vector<double> temp_lv;

    double _update_dynamic() override;
};


#endif //TRASIM_CFM_GIPPS_H
