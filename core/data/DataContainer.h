//
// Created by yzbyx on 2023/7/21.
//

#ifndef TRASIM_DATACONTAINER_H
#define TRASIM_DATACONTAINER_H

#include <map>
#include <string>
#include <set>
#include "../Constants.h"

class LaneAbstract;
class Vehicle;

class DataContainer {

public:
    LaneAbstract* lane;
    std::set<C_Info> save_info;
    std::vector<Vehicle*> total_car_list_has_data;
    std::map<C_Info, std::vector<double>> lane_total_data;

    void config(const std::set<C_Info>& save_info_ = {}, bool add_all = true);

    explicit DataContainer(LaneAbstract* lane = nullptr);

    void add_basic_info();

    std::map<C_Info, std::vector<double>> & get_lane_total_data();

private:
    std::vector<Vehicle *> & get_total_car_has_data();

};


#endif //TRASIM_DATACONTAINER_H
