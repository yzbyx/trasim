//
// Created by yzbyx on 2023/9/19.
//

#ifndef TRASIM_LANE_H
#define TRASIM_LANE_H


#include "../micro/LaneOpen.h"

class LaneOpen;

class Lane : public LaneOpen {
public:
    std::vector<LaneAbstract*> next_lanes;
//    Node node{};
};


#endif //TRASIM_LANE_H
