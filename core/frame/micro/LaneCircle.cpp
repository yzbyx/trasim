//
// Created by yzbyx on 2023/7/23.
//
#include "LaneCircle.h"
#include <algorithm>


LaneCircle::LaneCircle(double lane_length_, double speed_limit_) : LaneAbstract(lane_length_, speed_limit_) {
    this->is_circle = true;
}

void LaneCircle::step() {
    for (int i = 0; i < this->car_list.size(); ++i) {
        if (car_list[i]->is_cf_take_over) {
            continue;
        }
        this->car_list[i]->step(i);
    }
}

void LaneCircle::update_state() {
    for (auto & i : this->car_list) {
        this->car_state_update_common(i);

        if (i->x > this->lane_length) {
            i->x -= this->lane_length;
        }
    }

    std::sort(this->car_list.begin(), this->car_list.end(),
              [](const std::shared_ptr<Vehicle>& c1, const std::shared_ptr<Vehicle>& c2) {
        return c1->x < c2->x;
    });
}

void LaneCircle::car_summon() {
    LaneAbstract::car_summon();
}

LaneCircle::~LaneCircle() = default;
