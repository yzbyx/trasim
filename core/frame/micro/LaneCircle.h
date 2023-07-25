//
// Created by yzbyx on 2023/7/23.
//

#ifndef TRASIM_LANECIRCLE_H
#define TRASIM_LANECIRCLE_H

#include "LaneAbstract.h"

class LaneCircle : public LaneAbstract{

    void step() override;

    void update_state() override;

    void car_summon() override;

    ~LaneCircle() override;

public:
    explicit LaneCircle(double lane_length = 1000, double speed_limit_ = 30);
};

#endif //TRASIM_LANECIRCLE_H
