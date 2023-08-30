//
// Created by yzbyx on 2023/7/21.
//

#ifndef TRASIM_LCMODEL_KK_H
#define TRASIM_LCMODEL_KK_H


#include <random> // Include the <random> header for std::mt1993
#include <map>
#include "LCModel.h"

class LaneAbstract;

class LCModel_KK : public LCModel {
public:
    explicit LCModel_KK(Vehicle* vehicle, const std::map<std::string, double>& l_param);
private:
    double _a_0;
    double _delta_1;
    double _delta_2;
    double _gamma_ahead;
    double _gamma_behind;
    double _k;
    double _tau;
    double L_a_;
    double _p_0;
    double _lambda_b;
    double _delta_vr_1;

    double xm;

    static const std::map<std::string, double> default_f_param;

    LaneAbstract* left_lane;
    LaneAbstract* right_lane;
    LaneAbstract* lane{};

    void _update_dynamic() override;

    std::tuple<LaneAbstract *, double, double, double> base_cal() override;
    std::tuple<LaneAbstract *, double, double, double> on_ramp_cal() override;
    std::tuple<bool, double> safe_check(Vehicle* _f, Vehicle* _l);
    std::tuple<bool, double, double, double> safe_check_on_ramp(Vehicle* _f, Vehicle* _l);
    std::tuple<double, bool, double, double> safe_func_on_ramp_common(Vehicle *follower, Vehicle *leader, double v_hat) const;

    std::tuple<LaneAbstract *, double, double, double>
    step(int index, LaneAbstract* left_lane, LaneAbstract* right_lane) override;
};


#endif //TRASIM_LCMODEL_KK_H
