//
// Created by yzbyx on 2023/7/20.
//

#ifndef TRASIM_LCMODEL_H
#define TRASIM_LCMODEL_H

#include "../Model.h"
#include "../../Constants.h"

class LaneAbstract;

class LCModel : public Model {
public:
    explicit LCModel(Vehicle *vehicle);

    ~LCModel() override = default;

    /**
     *
     * @param index 车辆按照从下游到上游的顺序的index
     * @param left_lane 车辆当前车道的左侧车道
     * @param right_lane 车辆当前查到的右侧车道
     * @return 目标车道、x、v、a
     */
    virtual std::tuple<LaneAbstract *, double, double, double>
    step(int index, LaneAbstract *left_lane, LaneAbstract *right_lane) = 0;

    virtual std::tuple<LaneAbstract *, double, double, double> base_cal() = 0;

    virtual std::tuple<LaneAbstract *, double, double, double> on_ramp_cal() = 0;

    LCM name;
    std::mt19937 *random;
protected:
    double last_lc_time;
};

LCModel *get_lc_model(Vehicle *_driver, LCM name, const std::map<std::string, double> &param);


#endif //TRASIM_LCMODEL_H
