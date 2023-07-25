//
// Created by yzbyx on 2023/7/23.
//

#include "LaneOpen.h"
#include <iostream>
#include <algorithm>

LaneOpen::LaneOpen(double lane_length_, double speed_limit_) : LaneAbstract(lane_length_, speed_limit_) {
    this->is_circle = false;
    this->outflow_point = true;
    this->flow_rate = -1;
    this->thw_distribution = THW_DISTRIBUTION::Uniform;
    this->car_num_percent = {};
    this->next_car_time = 0;
    this->fail_summon_num = 0;
    this->offset_pos = 0;
}

void LaneOpen::car_loader(double flow_rate_, THW_DISTRIBUTION thw_distribution_, double offset_time, double offset_pos_) {
    this->flow_rate = flow_rate_ / 3600.;
    this->thw_distribution = thw_distribution_;
    this->next_car_time += offset_time;
    this->offset_pos = offset_pos_;

    if (!car_num_list.empty()) {
        car_num_percent = std::vector<double>(car_num_list.size());
        int sum_car_num = std::accumulate(this->car_num_list.begin(), this->car_num_list.end(), 0);
        for (int i = 0; i < this->car_num_list.size(); ++i) {
            car_num_percent[i] = (this->car_num_list[i] / (sum_car_num * 1.0));
        }
    }
}

void LaneOpen::step() {
    for (int i = 0; i < this->car_list.size(); ++i) {
        this->car_list[i]->step(i);
    }
}

void LaneOpen::car_summon() {
    if (0 < this->time_ && this->time_ < this->next_car_time) {
        return;
    }

    if (this->next_car_time <= this->time_) {
        if (car_num_percent.empty()) {
            throw std::runtime_error("未配置车辆，无法生成！");
        }
        if (!this->car_list.empty()) {
            Vehicle* first = this->car_list[0];
            if (first->x - first->length < 0) {
                this->fail_summon_num++;
                std::cout << "车道" << this->ID << "在" << this->step_ << "仿真步生成车辆失败！共延迟"
                          << this->fail_summon_num << "个仿真步" << std::endl;
                return;
            }
        }

        double i = RANDOM::DIS12(RANDOM::RND);
        int pos = -1;
        double sum_percent = 0;
        for (int j = 0; j < this->car_num_list.size(); ++j) {
            sum_percent += this->car_num_percent[j];
            if (i <= sum_percent) {
                pos = j;
                break;
            }
        }

        if (pos == -1) {
            std::cout << "车道" << this->ID << "在" << this->step_ << "仿真步生成车辆失败！" << std::endl;
            return;
        }

        auto* vehicle = new Vehicle(this, this->car_type_list[pos], this->get_new_car_id(), this->car_length_list[pos]);
        vehicle->x = this->offset_pos;
        vehicle->set_cf_model(this->cf_name_list[pos], this->cf_param_list[pos]);
        vehicle->set_lc_model(this->lc_name_list[pos], this->lc_param_list[pos]);

        double initial_speed = this->car_initial_speed_list[pos];
        if (initial_speed >= 0) {
            double min_speed = std::max(initial_speed - 0.5, 0.0);
            double max_speed = initial_speed + 0.5;
            double speed = this->speed_with_random_list[pos] ? (RANDOM::DIS12(RANDOM::RND)) * (max_speed - min_speed) + min_speed : initial_speed;
            vehicle->v = speed;
        } else {
            if (initial_speed == -1) {
                if (this->car_list.empty()) {
                    vehicle->v = vehicle->cf_model->get_expect_speed();
                } else {
                    vehicle->v = this->car_list[0]->v;
                }
            } else {
                if (!this->car_list.empty()) {
                    Vehicle* leader = this->car_list[0];
                    vehicle->v = leader->v;
                    double l_d = leader->dhw();
                    if (!std::isnan(l_d)) {
                        double x = leader->x - l_d - leader->length;
                        vehicle->x = (x >= vehicle->x) ? x : vehicle->x;
                    }
                } else {
                    vehicle->v = vehicle->cf_model->get_expect_speed();
                }
            }
        }

        vehicle->a = 0;
        vehicle->set_car_param(this->car_param_list[pos]);

        if (!this->car_list.empty()) {
            vehicle->leader = this->car_list[0];
            this->car_list[0]->follower = vehicle;
        }

        this->car_list.insert(this->car_list.begin(), vehicle);

        this->_set_next_summon_time();
    }
}

void LaneOpen::update_state() {
    for (auto & i : this->car_list) {
        this->car_state_update_common(i);

        if (i->x > this->lane_length) {
            i->is_run_out = true;
            this->car_remove(i, i->has_data());
        }
    }
}

void LaneOpen::_set_next_summon_time() {
    double thw;
    if (this->thw_distribution == THW_DISTRIBUTION::Uniform) {
        thw = (this->flow_rate > 0) ? (1 / this->flow_rate) : std::numeric_limits<double>::infinity();
    } else if (this->thw_distribution == THW_DISTRIBUTION::Exponential) {
        double a = RANDOM::DIS12(RANDOM::RND);
        thw = (this->flow_rate > 0) ? (-std::log(a) / this->flow_rate) : std::numeric_limits<double>::infinity();
    } else {
        thw = (this->flow_rate > 0) ? (1 / this->flow_rate) : std::numeric_limits<double>::infinity();
    }
    this->next_car_time += thw;
}

LaneOpen::~LaneOpen() = default;
