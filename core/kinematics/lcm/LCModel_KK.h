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
    explicit LCModel_KK(const std::shared_ptr<Vehicle>& vehicle, const std::map<std::string, double>& l_param_);
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

    std::shared_ptr<LaneAbstract> left_lane;
    std::shared_ptr<LaneAbstract> right_lane;
    std::shared_ptr<LaneAbstract> lane{};

    double _update_dynamic() override;

    std::tuple<std::shared_ptr<LaneAbstract>, double, double, double> base_cal() override;
    std::tuple<std::shared_ptr<LaneAbstract>, double, double, double> on_ramp_cal() override;
    std::tuple<bool, double> safe_check(const std::shared_ptr<Vehicle>& _f, const std::shared_ptr<Vehicle>& _l);
    std::tuple<bool, double, double, double> safe_check_on_ramp(const std::shared_ptr<Vehicle>& _f, const std::shared_ptr<Vehicle>& _l);
    std::tuple<double, bool, double, double> safe_func_on_ramp_common(const std::shared_ptr<Vehicle>& follower, const std::shared_ptr<Vehicle>& leader, double v_hat) const;

    std::tuple<std::shared_ptr<LaneAbstract>, double, double, double>
    step(int index, std::shared_ptr<LaneAbstract> left_lane_, std::shared_ptr<LaneAbstract> right_lane_) override;
};


#endif //TRASIM_LCMODEL_KK_H
