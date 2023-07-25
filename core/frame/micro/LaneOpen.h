//
// Created by yzbyx on 2023/7/23.
//

#ifndef TRASIM_LANEOPEN_H
#define TRASIM_LANEOPEN_H

#include "LaneAbstract.h"


class LaneOpen : public LaneAbstract{
public:
    explicit LaneOpen(double lane_length_ = 1000, double speed_limit_ = 30);
    void car_loader(double flow_rate_, THW_DISTRIBUTION thw_distribution_,
                    double offset_time, double offset_pos_) override;
    void step() override;
    void car_summon() override;
    void update_state() override;
    ~LaneOpen() override;

private:
    void _set_next_summon_time();

    bool outflow_point;
    double flow_rate;
    THW_DISTRIBUTION thw_distribution;
    std::vector<double> car_num_percent;
    double next_car_time;
    int fail_summon_num;
    double offset_pos;
};


#endif //TRASIM_LANEOPEN_H
