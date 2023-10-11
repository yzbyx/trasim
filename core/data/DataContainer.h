//
// Created by yzbyx on 2023/7/21.
//

#ifndef TRASIM_DATACONTAINER_H
#define TRASIM_DATACONTAINER_H

#include <map>
#include <string>
#include <set>
#include <memory>
#include "../Constants.h"

class LaneAbstract;
class Vehicle;

struct C_InfoComparator {
    bool operator()(C_Info lhs, C_Info rhs) const {
        return static_cast<int>(lhs) < static_cast<int>(rhs);
    }
};

class DataContainer {

public:
    std::shared_ptr<LaneAbstract> lane;
    std::set<C_Info, C_InfoComparator> save_info = {};
    std::vector<std::shared_ptr<Vehicle>> total_car_list_has_data;
    std::map<C_Info, std::vector<double>> lane_total_data;

    void config(const std::set<C_Info>& save_info_ = {}, bool add_all = true);

    explicit DataContainer(std::shared_ptr<LaneAbstract> lane = nullptr);

    void add_basic_info();

    std::map<C_Info, std::vector<double>> & get_lane_total_data();

private:
    std::vector<std::shared_ptr<Vehicle>> & get_total_car_has_data();

};


#endif //TRASIM_DATACONTAINER_H
