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
#ifdef WIN32
    std::set<C_Info> basic_save_info = {C_Info::lane_add_num, C_Info::step, C_Info::time,
                                  C_Info::id, C_Info::leader_id, C_Info::x, C_Info::v, C_Info::a, C_Info::l};
    save_info.insert(basic_save_info.begin(), basic_save_info.end());
#else
    save_info.insert({C_Info::lane_add_num, C_Info::step, C_Info::time,
                      C_Info::id, C_Info::leader_id, C_Info::x, C_Info::v, C_Info::a, C_Info::l});
#endif
}

/**
 * Lane级别的数据获取
 * @param path_
 */
std::map<C_Info, std::vector<double>> &DataContainer::get_lane_total_data() {
    get_total_car_has_data();
    for (auto name: save_info) {
        lane_total_data[name] = {};
        for (const auto& car: total_car_list_has_data) {
            std::vector<double> temp = car->get_data_list(name);
            lane_total_data[name].insert(lane_total_data[name].end(), temp.begin(), temp.end());
        }
    }
    return lane_total_data;
}

std::vector<std::shared_ptr<Vehicle>> & DataContainer::get_total_car_has_data() {
    if (total_car_list_has_data.empty()) {
        std::vector<std::shared_ptr<Vehicle>> car_on_lane_has_data = {};
        for (const auto& car: lane->car_list) {
            if (car->has_data()) {
                car_on_lane_has_data.push_back(car);
            }
        }
        total_car_list_has_data.insert(
                total_car_list_has_data.end(),
                lane->out_car_has_data.begin(),
                lane->out_car_has_data.end());
        total_car_list_has_data.insert(
                total_car_list_has_data.end(),
                car_on_lane_has_data.begin(),
                car_on_lane_has_data.end());
    }
    return total_car_list_has_data;
}

DataContainer::DataContainer(std::shared_ptr<LaneAbstract> lane_) {
    this->lane = lane_;
    save_info = {};
    total_car_list_has_data = {};
    lane_total_data = {};
}
