//
// Created by yzbyx on 2023/7/21.
//

#include "DataContainer.h"
#include "../Vehicle.h"
#include "../frame/micro/LaneAbstract.h"


void DataContainer::config(const std::set<C_Info> &save_info_, bool add_all) {
    if (add_all) {
        save_info.insert(ALL_C_INFO.begin(), ALL_C_INFO.end());
        return;
    }
    save_info.insert(save_info_.begin(), save_info_.end());
}

void DataContainer::add_basic_info() {
    save_info.insert({C_Info::lane_add_num, C_Info::id, C_Info::car_type, C_Info::time, C_Info::step});
}

/**
 * Lane级别的数据获取
 * @param path_
 */
std::map<C_Info, std::vector<double>> &DataContainer::get_lane_total_data() {
    get_total_car_has_data();
    for (auto name: save_info) {
        lane_total_data[name] = {};
        for (auto car: total_car_list_has_data) {
            std::vector<double> temp = car->get_data_list(name);
            lane_total_data[name].insert(lane_total_data[name].end(), temp.begin(), temp.end());
        }
    }
    return lane_total_data;
}

std::vector<Vehicle *> &DataContainer::get_total_car_has_data() {
    if (total_car_list_has_data.empty()) {
        std::vector<Vehicle *> car_on_lane_has_data = {};
        for (auto car: lane->car_list) {
            if (car->has_data()) {
                car_on_lane_has_data.push_back(car);
            }
        }
        total_car_list_has_data.insert(
                total_car_list_has_data.end(),
                car_on_lane_has_data.begin(),
                car_on_lane_has_data.end());
    }
    return total_car_list_has_data;
}

DataContainer::DataContainer(LaneAbstract *lane_) {
    lane = lane_;
    save_info = {};
    total_car_list_has_data = {};
    lane_total_data = {};
}
