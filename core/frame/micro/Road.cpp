//
// Created by yzbyx on 2023/7/21.
//

#include "Road.h"
#include "LaneCircle.h"
#include "LaneOpen.h"
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <algorithm>
#include <numeric>
#include <thread>

Road::Road(double length_, int id) {
    lane_length = length_;
    id_accumulate = 0;
    step_ = 0;
    sim_step = 0;
    dt = 0;
    time_ = 0;
    ID = id;
    road_total_data = {};
    data_processor = std::make_shared<DataProcessor>();
    has_ui = false;
    frame_rate = -1;
}

std::vector<std::shared_ptr<LaneAbstract>> Road::add_lanes(int lane_num, bool is_circle, const std::vector<int> &real_index) {
    std::vector<std::shared_ptr<LaneAbstract>> lanes;
    std::vector<int> lane_indices(lane_num);
    std::iota(lane_indices.begin(), lane_indices.end(), 0);
    std::vector<int> real_index_copy = (real_index.empty()) ? lane_indices : real_index;
    if (!real_index.empty() && lane_indices.size() != real_index.size()) {
        throw std::runtime_error("real_index's length error!");
    }

    for (int i = 0; i < lane_num; ++i) {
        std::shared_ptr<LaneAbstract> lane;
        if (is_circle) {
            lane = std::make_shared<LaneCircle>(this->lane_length);
        } else {
            lane = std::make_shared<LaneOpen>(this->lane_length);
        }
        lane->index = real_index_copy[i];
        lane->add_num = static_cast<int>(lanes.size());
        lane->ID = std::to_string(lane->index).append("-").append(std::to_string(lane->add_num));
        lane->road_control = true;
        lane->road = shared_from_this();
        lane->left_neighbour_lanes = std::vector<std::shared_ptr<LaneAbstract>>();
        lane->right_neighbour_lanes = std::vector<std::shared_ptr<LaneAbstract>>();
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


void Road::run_config(bool data_save, bool has_ui_, double dt_, int sim_step_, int frame_rate_, int warm_up_step) {
    this->dt = dt_;
    this->sim_step = sim_step_;
    has_ui = has_ui_;
    frame_rate = frame_rate_;

    if (has_ui) {
        ui = std::make_shared<UI>(shared_from_this());
//        std::thread draw_thread(ui_thread, std::ref(ui));
    }

    for (const auto& lane: this->lane_list) {
        lane->run_config(data_save, UpdateMethod::Euler,
                         warm_up_step, dt_, sim_step_, false);
        auto lane_it = lane->begin();
        lanes_iter.push_back(lane_it);
    }
}

void Road::run(bool data_save, bool has_ui_, double dt_, int sim_step_, int frame_rate_, int warm_up_step) {
    run_config(data_save, has_ui_, dt_, sim_step_, frame_rate_, warm_up_step);

    auto beforeTime = std::chrono::steady_clock::now();

    while (this->sim_step != this->step_) {
        run_first_part();
        run_second_part();
    }

    auto afterTime = std::chrono::steady_clock::now();
    double duration_second = std::chrono::duration<double>(afterTime - beforeTime).count();
    std::cout << "[Road_run] time usage: " << duration_second << "s" << std::endl;
}

void Road::run_first_part() {
    // stage=1时做的事
    for (auto &lane_it: lanes_iter) {
        ++lane_it;  // 更新跟驰
    }
    for (auto &lane_it: lanes_iter) {
        ++lane_it;  // 生成车辆、记录状态、计算跟驰
    }
}

void Road::run_second_part() {
    // stage=2时做的事
    // 由于车辆换道会改变其他车道的车辆分布，因此需要在完成单个车道的车辆换道模型计算后，
    // 接着进行车辆位置的更新，避免多车道同时位置更新导致的换道冲突
    for (const auto& lane: lane_list) {
        Road::step_lane_change(lane);
        Road::update_lc_state(lane); // 换道状态更新
    }
    this->step_ += 1;
    this->time_ += this->dt;
    if (this->has_ui) { this->ui->ui_update(); }
}

void Road::step_lane_change(const std::shared_ptr<LaneAbstract>& lane) {
    for (int i = 0; i < lane->car_list.size(); ++i) {
        for (const auto& car: lane->car_list) {
            if (car->type != VType::OBSTACLE && car->lc_model != nullptr && !car->is_lc_take_over) {
                // 耗时热点get_available_adjacent_lane
                auto result = Road::get_available_adjacent_lane(lane, car->x, car->type);
                car->step_lane_change(
                        i,
                        std::get<0>(result),
                        std::get<1>(result),
                        std::get<2>(result)
                );
            }
        }
    }
}

void Road::update_lc_state(const std::shared_ptr<LaneAbstract>& lane) {
    std::shared_ptr<Vehicle> car_lc_last = nullptr;  // 该车道最近一次换道的车辆
    std::vector<std::shared_ptr<Vehicle>> car_list = lane->car_list;
    std::reverse(car_list.begin(), car_list.end());
    for (const auto& car: car_list) {
        if (car->type != VType::OBSTACLE) {
            if (car->lc_target_lane != car->lane and car->lc_target_lane != nullptr) {
                if (Road::check_and_correct_lc_pos(car->lc_target_lane, car_lc_last, car)) {
                    lane->car_remove(car, false, true);
                    car->x = std::get<1>(car->lc_result);
                    car->v = std::get<2>(car->lc_result);
                    car->a = std::get<3>(car->lc_result);
                    car->lc_target_lane->car_insert_by_instance(car, false);
                    car_lc_last = car;
                }
                car->lc_target_lane = nullptr;
                car->is_lc_take_over = false;
            }
        }
    }
}

bool Road::check_and_correct_lc_pos(const std::shared_ptr<LaneAbstract>& target_lane,
                                    const std::shared_ptr<Vehicle>& car_lc_last,
                                    const std::shared_ptr<Vehicle>& car) {
    double target_pos = std::get<1>(car->lc_result);
    if (target_lane->is_circle && target_pos > target_lane->lane_length) {
        target_pos -= target_lane->lane_length;
        std::get<1>(car->lc_result) = target_pos;
    }

    if (car_lc_last == nullptr || car_lc_last->lane != target_lane) {
        return true;
    }
    double dist = car_lc_last->get_dist(target_pos);
    if (dist > car->length || dist < - car_lc_last->length) {
        return true;
    }

    return false;
}

std::tuple<std::shared_ptr<LaneAbstract>, std::shared_ptr<LaneAbstract>, std::vector<SECTION_TYPE>>
Road::get_available_adjacent_lane(const std::shared_ptr<LaneAbstract>& lane, double pos, VType car_type) {
    std::vector<std::shared_ptr<LaneAbstract>> lefts = lane->left_neighbour_lanes;
    std::vector<std::shared_ptr<LaneAbstract>> rights = lane->right_neighbour_lanes;

    std::vector<SECTION_TYPE> section_type = lane->get_section_type(pos, car_type);
    std::vector<CONTROL_TYPE> control_type = lane->get_control_type(pos, car_type);
    if (control_type[0] == CONTROL_TYPE::NO_LIMIT) {
        if (lefts.empty() && rights.empty()) {
            return {nullptr, nullptr, section_type};
        } else if (lefts.empty()) {
            return {nullptr, rights[0], section_type};
        } else if (rights.empty()) {
            return {lefts[0], nullptr, section_type};
        }
    }

    if (std::find(control_type.begin(), control_type.end(), CONTROL_TYPE::NO_LEFT) != control_type.end()) {
        lefts = std::vector<std::shared_ptr<LaneAbstract>>{nullptr};
    } else {
        for (int i = static_cast<int>(lefts.size()) - 1; i >= 0; --i) {
            std::vector<CONTROL_TYPE> temp = lefts[i]->get_control_type(pos, car_type);
            if (std::find(temp.begin(), temp.end(), CONTROL_TYPE::NO_RIGHT_CAR) != temp.end()) {
                lefts.erase(lefts.begin() + i);
            }
        }
        if (lefts.empty()) {
            lefts = std::vector<std::shared_ptr<LaneAbstract>>{nullptr};
        }
    }

    if (std::find(control_type.begin(), control_type.end(), CONTROL_TYPE::NO_RIGHT) != control_type.end()) {
        rights = std::vector<std::shared_ptr<LaneAbstract>>{nullptr};
    } else {
        for (int i = static_cast<int>(rights.size()) - 1; i >= 0; --i) {
            std::vector<CONTROL_TYPE> temp = lefts[i]->get_control_type(pos, car_type);
            if (std::find(temp.begin(), temp.end(), CONTROL_TYPE::NO_LEFT_CAR) != temp.end()) {
                lefts.erase(lefts.begin() + i);
            }
        }
        if (rights.empty()) {
            rights = std::vector<std::shared_ptr<LaneAbstract>>{nullptr};
        }
    }

    if (!(lefts.size() == 1 && rights.size() == 1)) {
        throw std::runtime_error("There are overlapping lanes!");
    }
    return {lefts[0], rights[0], section_type};
}

double Road::get_car_info(int car_id, C_Info info, int lane_add_num) {
    if (lane_add_num == -1) {
        for (const auto& lane: this->lane_list) {
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
    std::cout << "Vehicle not found!" << std::endl;
    return NAN;
}

int Road::car_insert_middle(int lane_add_num, int front_car_id, VehicleData &data) {
    return lane_list[lane_add_num]->car_insert_middle(front_car_id, data);
}

void Road::cf_take_over(int car_id, double acc_values) {
    for (const auto& lane: this->lane_list) {
        for (const auto& car: lane->car_list) {
            if (car->ID == car_id) {
                car->cf_acc = acc_values;
                car->is_cf_take_over = true;
                break;
            }
        }
    }
}

void Road::lc_take_over(int car_id, const std::tuple<std::shared_ptr<LaneAbstract>,
        double, double, double> &lc_result) {
    for (const auto& lane: this->lane_list) {
        for (const auto& car: lane->car_list) {
            if (car->ID == car_id) {
                car->lc_result = lc_result;
                car->is_lc_take_over = true;
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

    return {-1};
}

Road::~Road() {
}

std::map<C_Info, std::vector<double>> &Road::get_road_total_data() {
    if (road_total_data.empty()) {
        for (const auto& lane: this->lane_list) {
            lane->data_container->get_lane_total_data();
            for (auto name: lane->data_container->save_info) {
                road_total_data[name].insert(road_total_data[name].end(),
                                             lane->data_container->lane_total_data[name].begin(),
                                             lane->data_container->lane_total_data[name].end());
            }
        }
    }
    return road_total_data;
}

int Road::get_car_num_on_road() {
    int num = 0;
    for (const auto& lane: lane_list) {
        num += lane->car_num();
    }
    return num;
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
