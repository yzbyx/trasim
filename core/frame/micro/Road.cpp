//
// Created by yzbyx on 2023/7/21.
//

#include "Road.h"
#include "LaneCircle.h"
#include "LaneOpen.h"
#include "../../../util/timer.h"
#include <iostream>
#include <limits>
#include <string>
#include <algorithm>

Road::Road(double length_, int id){
    lane_length = length_;
    id_accumulate = 0;
    step_ = 0;
    sim_step = 0;
    dt = 0;
    time_ = 0;
    ID = id;
    road_total_data = {};
    data_processor = new DataProcessor();
}

std::vector<LaneAbstract*> Road::add_lanes(int lane_num, bool is_circle, const std::vector<int>& real_index) {
    std::vector<LaneAbstract*> lanes;
    std::vector<int> lane_indices(lane_num);
    std::iota(lane_indices.begin(), lane_indices.end(), 0);
    std::vector<int> real_index_copy = (real_index.empty()) ? lane_indices : real_index;
    if (!real_index.empty() && lane_indices.size() != real_index.size()) {
        throw std::runtime_error("real_index长度错误！");
    }

    for (int i = 0; i < lane_num; ++i) {
        LaneAbstract* lane;
        if (is_circle) {
            lane = new LaneCircle(this->lane_length);
        } else {
            lane = new LaneOpen(this->lane_length);
        }
        lane->index = real_index_copy[i];
        lane->add_num = static_cast<int>(lanes.size());
        lane->ID = std::to_string(lane->index).append("-").append(std::to_string(lane->add_num));
        lane->road_control = true;
        lane->road = this;
        lane->left_neighbour_lanes = std::vector<LaneAbstract*>();
        lane->right_neighbour_lanes = std::vector<LaneAbstract*>();
        lanes.push_back(lane);
    }

    // 计算各车道的邻近车道
    for (int i = 0; i < lanes.size(); ++i) {
        for (int j = 0; j < lanes.size(); ++j) {
            if (lanes[i]->index - 1 == lanes[j]->index) {
                lanes[i]->left_neighbour_lanes.push_back(lanes[j]);
            } else if (lanes[i]->index + 1 == lanes[j]->index) {
                lanes[i]->right_neighbour_lanes.push_back(lanes[j]);
            }
        }
    }
    this->lane_list.insert(this->lane_list.end(), lanes.begin(), lanes.end());
    return lanes;
}


void Road::run_config(bool data_save, bool has_ui, double dt_, int sim_step_, int frame_rate, int warm_up_step) {
    this->dt = dt_;
    this->sim_step = sim_step_;

    for (auto lane : this->lane_list) {
        lane->run_config(data_save, UpdateMethod::Euler,
                         warm_up_step, dt_, sim_step_, false);
        auto lane_it = lane->begin();
        lanes_iter.push_back(lane_it);
    }
}

void Road::run(bool data_save, bool has_ui, double dt_, int sim_step_, int frame_rate, int warm_up_step) {
    this->dt = dt_;
    this->sim_step = sim_step_;

    if (has_ui) {

    }

    for (auto lane : this->lane_list) {
        lane->run_config(data_save, UpdateMethod::Euler,
                         warm_up_step, dt_, sim_step_, false);
        auto lane_it = lane->begin();
        lanes_iter.push_back(lane_it);
    }

    time_t timeIn = time(nullptr);
    std::string timeStart = get_current_time();

    while (this->sim_step != this->step_) {
        run_first_part();
        run_second_part();
    }

    time_t timeOut = time(nullptr);
    std::string log_string = std::string("[Road_run] time usage: ") + timeStart + " + " +
                             std::to_string((timeOut - timeIn) * 1000L / 1000.0L) + " s";
    std::cout << log_string << std::endl;
}

void Road::run_first_part() {
    // 计算跟驰
    for (auto& lane_it : lanes_iter) {
        ++lane_it;
    }
    // 更新跟驰&生成车辆
    for (auto& lane_it : lanes_iter) {
        ++lane_it;
    }
    this->step_lane_change();
}

void Road::run_second_part() {
    this->update_lc_state(); // 换道状态更新
    this->step_ += 1;
    this->time_ += this->dt;
//    if (this->has_ui) { this->ui.ui_update(); }
}

void Road::step_lane_change() {
    for (auto lane : this->lane_list) {
        for (int i = 0; i < lane->car_list.size(); ++i) {
            if (lane->car_list[i]->type != VType::OBSTACLE) {
                if (lane->car_list[i]->lc_model != nullptr) {
                    LaneAbstract* left, *right;
                    std::tie(left, right) = Road::get_available_adjacent_lane(lane, lane->car_list[i]->x, lane->car_list[i]->type);
                    lane->car_list[i]->step_lane_change(i, left, right);
                }
            }
        }
    }
}

void Road::update_lc_state() {
    for (auto lane : this->lane_list) {
        Vehicle* car_lc_last = nullptr;
        std::vector<Vehicle*> car_list = lane->car_list;
        std::reverse(car_list.begin(), car_list.end());
        for (auto car : car_list) {
            if (car->type != VType::OBSTACLE) {
                int lc = static_cast<int>(std::round(car->lc_result["lc"]));
                if (lc != 0) {
                    LaneAbstract* target_lane = this->lane_list[car->lane->index + lc];
                    if (Road::check_and_correct_lc_pos(target_lane, car_lc_last, car)) {
                        lane->car_remove(car, false);
                        car->x = car->lc_result["x"];
                        car->v = car->lc_result["v"];
                        car->a = car->lc_result["a"];
                        target_lane->car_insert_by_instance(car, false);
                        car_lc_last = car;
                    }
                    car->lc_result.clear();
                }
            }
        }
    }
}

bool Road::check_and_correct_lc_pos(LaneAbstract* target_lane, Vehicle* car_lc_last, Vehicle* car) {
    double target_pos = car->lc_result["x"];
    if (target_lane->is_circle && target_pos > target_lane->lane_length) {
        car->lc_result["x"] -= target_lane->lane_length;
    }
    if (car_lc_last == nullptr || car_lc_last->lane != target_lane) {
        return true;
    }
    double dist = car_lc_last->get_dist(target_pos);
    if (dist > car->length || dist < -car_lc_last->length) {
        return true;
    }
    return false;
}

std::pair<LaneAbstract*, LaneAbstract*> Road::get_available_adjacent_lane(LaneAbstract* lane, double pos, VType car_type) {
    std::vector<LaneAbstract*> lefts = lane->left_neighbour_lanes;
    std::vector<LaneAbstract*> rights = lane->right_neighbour_lanes;
    std::set<SECTION_TYPE> section_type = lane->get_section_type(pos, car_type);

    if (section_type.count(SECTION_TYPE::NO_LEFT) > 0) {
        lefts = std::vector<LaneAbstract*>{nullptr};
    } else {
        for (int i = static_cast<int>(lefts.size()) - 1; i >= 0; --i) {
            if (lefts[i]->get_section_type(pos, car_type).count(SECTION_TYPE::NO_RIGHT_CAR) > 0) {
                lefts.erase(lefts.begin() + i);
            }
        }
        if (lefts.empty()) {
            lefts = std::vector<LaneAbstract*>{nullptr};
        }
    }

    if (section_type.count(SECTION_TYPE::NO_RIGHT) > 0) {
        rights = std::vector<LaneAbstract*>{nullptr};
    } else {
        for (int i = static_cast<int>(rights.size()) - 1; i >= 0; --i) {
            if (rights[i]->get_section_type(pos, car_type).count(SECTION_TYPE::NO_LEFT_CAR) > 0) {
                rights.erase(rights.begin() + i);
            }
        }
        if (rights.empty()) {
            rights = std::vector<LaneAbstract*>{nullptr};
        }
    }

    if (!(lefts.size() == 1 && rights.size() == 1)) {
        throw std::runtime_error("有重叠车道！");
    }
    return {lefts[0], rights[0]};
}

double Road::get_car_info(int car_id, C_Info info, int lane_add_num) {
    if (lane_add_num == -1) {
        for (auto lane : this->lane_list) {
            double result = lane->get_car_info(car_id, info);
            if (result != NAN) {
                return result;
            }
        }
    } else {
        double result = this->lane_list[lane_add_num]->get_car_info(car_id, info);
        if (result != NAN) {
            return result;
        }
    }
    std::cout << "未找到车辆！" << std::endl;
}

int Road::car_insert_middle(int lane_add_num, int front_car_id, VehicleData & data) {
    return lane_list[lane_add_num]->car_insert_middle(front_car_id, data);
}

void Road::take_over(int car_id, double acc_values, const std::map<std::string, double>& lc_result) {
    for (auto lane : this->lane_list) {
        for (auto car : lane->car_list) {
            if (car->ID == car_id) {
                car->cf_acc = acc_values;
                car->lc_result = lc_result;
                break;
            }
        }
    }
}

int Road::get_new_car_id() {
    id_accumulate += 1;
    return id_accumulate;
}

int Road::get_appropriate_car(int lane_add_num) {
    return this->lane_list[lane_add_num]->get_appropriate_car();
}

std::vector<int> Road::find_on_lanes(int car_id) {
//    data_to_df();
}

Road::~Road() {
    for (auto lane : lane_list) {
        delete lane;
    }
    delete data_processor;
}

std::map<C_Info, std::vector<double>> & Road::get_road_total_data() {
    if (road_total_data.empty()) {
        for (auto lane : this->lane_list) {
            lane->data_container->get_lane_total_data();
            for (auto name : lane->data_container->save_info) {
                road_total_data[name].insert(road_total_data[name].end(),
                                             lane->data_container->lane_total_data[name].begin(),
                                             lane->data_container->lane_total_data[name].end());
            }
        }
    }
    return road_total_data;
}

std::pair<int, int> Road::RoadIterator::operator*() {
    return {ptr->step_, stage};
}

Road::RoadIterator::RoadIterator(Road *p) {
    ptr = p;
    stage = 1;
}

Road::RoadIterator &Road::RoadIterator::operator++() {
    if (stage == 1) {
        ptr->run_first_part();
        stage = 2;
    } else {
        ptr->run_second_part();
        stage = 1;
    }

    return *this;
}

bool Road::RoadIterator::operator!=(const Road::RoadIterator &other) const {
    return ptr->step_ != ptr->sim_step;
}
