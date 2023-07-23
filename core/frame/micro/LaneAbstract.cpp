//
// Created by yzbyx on 2023/7/20.
//

#include <algorithm>
#include <random>
#include "LaneAbstract.h"


LaneAbstract::LaneAbstract(float lane_length_, float speed_limit_) {
    ID = 0;
    index = 0;
    add_num = 0;
    default_speed_limit = speed_limit_;
    car_num_total = 0;
    is_circle = false;
    lane_length = lane_length_;
    id_accumulate = 0;
    step_ = 0;
    time_ = 0;
    yield_ = true;
    road_control = false;
    force_speed_limit = false;
    state_update_method = UpdateMethod::Euler;
    dt = 0.1;
    warm_up_step = static_cast<int>(5 * 60 / dt);
    sim_step = static_cast<int>(10 * 60 / dt);
    data_save = false;
}

int LaneAbstract::get_new_car_id() {
    if (!road_control) {
        id_accumulate += 1;
        return id_accumulate;
    } else {
        return road->get_new_car_id();
    }
}

void LaneAbstract::set_section_type(SECTION_TYPE type_, double start_pos, double end_pos,
                                    std::vector<VType> car_types) {
    if (start_pos < 0) {
        start_pos = 0;
    }
    if (end_pos < 0) {
        end_pos = lane_length;
    }

    if (car_types.empty()) {
        car_types = ALL_V_TYPE;
    }

    for (VType car_type : car_types) {
        if (section_type.find(car_type) != section_type.end()) {
            section_type[car_type].insert({type_, {start_pos, end_pos}});
        } else {
            section_type.insert({car_type, {{type_, {start_pos, end_pos}}}});
        }
    }
}

std::set<SECTION_TYPE> LaneAbstract::get_section_type(double pos, VType car_type) {
    std::set<SECTION_TYPE> type_ = {};
    if (section_type.empty()) {
        type_.insert(SECTION_TYPE::BASE);
    }

    auto section_type_for_type = section_type.find(car_type);
    if (section_type_for_type != section_type.end()) {
        for (const auto& [key, pos_] : section_type_for_type->second) {
            if ((pos_[0] <= pos) && (pos < pos_[1])) {
                type_.insert(key);
            }
            if (pos == lane_length && pos == pos_[1]) {
                type_.insert(key);
            }
        }
    }
    return type_;
}

void LaneAbstract::set_speed_limit(double speed_limit_, double start_pos, double end_pos,
                                    std::vector<VType> car_types) {
    if (start_pos < 0) {
        start_pos = 0;
    }
    if (end_pos < 0) {
        end_pos = lane_length;
    }

    if (car_types.empty()) {
        car_types = ALL_V_TYPE;
    }

    for (VType car_type : car_types) {
        if (speed_limit.find(car_type) != speed_limit.end()) {
            speed_limit[car_type].insert({speed_limit_, {start_pos, end_pos}});
        } else {
            speed_limit.insert({car_type, {{speed_limit_, {start_pos, end_pos}}}});
        }
    }
}

double LaneAbstract::get_speed_limit(double pos, VType car_type) {
    if (speed_limit.empty()) {
        return default_speed_limit;
    }

    auto section_type_for_type = speed_limit.find(car_type);
    if (section_type_for_type != speed_limit.end()) {
        for (const auto& [key, pos_] : section_type_for_type->second) {
            if ((pos_[0] <= pos) && (pos < pos_[1])) {
                return key;
            }
        }
    }
    return default_speed_limit;
}

int LaneAbstract::car_num() const {
    return static_cast<int>(car_list.size());
}

void LaneAbstract::car_config(float car_num, float car_length, VType car_type, float car_initial_speed,
                              bool speed_with_random,
                              CFM cf_name,
                              const std::map<std::string, double>& cf_param_,
                              const std::map<std::string, double>& car_param_,
                              LCM lc_name,
                              const std::map<std::string, double>& lc_param_) {
    // 如果是开边界，则car_num与car_loader配合可以代表车型比例，如果car_loader中的flow为复数，则car_num为真实生成车辆数
    if (0 < car_num && car_num < 1) {
        car_num = static_cast<float>(std::floor(this->lane_length * car_num / car_length));
    }
    this->car_num_list.push_back(static_cast<int>(car_num));
    this->car_length_list.push_back(car_length);
    this->car_type_list.push_back(car_type);
    this->car_initial_speed_list.push_back(car_initial_speed);
    this->speed_with_random_list.push_back(speed_with_random);
    this->cf_name_list.push_back(cf_name);
    this->cf_param_list.push_back(cf_param_);
    this->car_param_list.push_back(car_param_);
    this->lc_name_list.push_back(lc_name);
    this->lc_param_list.push_back(lc_param_);
}

std::vector<int> LaneAbstract::car_load(float car_gap, int jam_num) {
    int car_num_total_ = std::accumulate(this->car_num_list.begin(), this->car_num_list.end(), 0);
    double car_length_total = std::inner_product(this->car_num_list.begin(), this->car_num_list.end(),
                                                this->car_length_list.begin(), 0.);
    double gap = (this->lane_length - car_length_total) / car_num_total_;
    if (gap < 0) {
        throw std::runtime_error("该密度下，车辆重叠！");
    }

    double x = 0;
    int car_count = 0;
    std::vector<int> car_type_index_list;
    for (size_t i = 0; i < this->car_num_list.size(); ++i) {
        car_type_index_list.insert(car_type_index_list.end(), this->car_num_list[i], static_cast<int>(i));
    }
    std::shuffle(car_type_index_list.begin(), car_type_index_list.end(), RANDOM::RND);

    std::vector<int> num_node;  // 记录分段阻塞对应的头车index
    double jam_gap = 0;
    if (jam_num > 1) {
        int car_num = static_cast<int>(std::floor(car_num_total / jam_num));
        jam_gap = (this->lane_length - car_length_total) / jam_num;
        for (int j = 1; j < jam_num; ++j) {
            num_node.push_back(car_num * j);
        }
    }

    for (size_t index_ = 0; index_ < car_type_index_list.size(); ++index_) {
        int i = car_type_index_list[index_];
        auto* vehicle = new Vehicle{this, this->car_type_list[i], this->get_new_car_id(), this->car_length_list[i]};
        vehicle->set_cf_model(this->cf_name_list[i], this->cf_param_list[i]);
        vehicle->set_lc_model(this->lc_name_list[i], this->lc_param_list[i]);
        if (this->car_initial_speed_list[i] < 0) {
            this->car_initial_speed_list[i] = vehicle->cf_model->get_expect_speed();
        }
        vehicle->x = x;
        vehicle->v = (speed_with_random_list[i] ? std::uniform_real_distribution<double>(
                std::max(this->car_initial_speed_list[i] - 0.5, 0.0),
                this->car_initial_speed_list[i] + 0.5)(RANDOM::RND)
                                                     : this->car_initial_speed_list[i]);
        vehicle->a = 0;
        vehicle->set_car_param(this->car_param_list[i]);

        this->car_list.push_back(vehicle);

        if (index_ != static_cast<size_t>(car_num_total - 1)) {
            double length = car_length_list[static_cast<size_t>(car_type_index_list[index_ + 1])];
            if (jam_num > 1) {
                if (!num_node.empty() && num_node.front() == car_count) {
                    x = x + jam_gap + length;
                    num_node.erase(num_node.begin());
                } else {
                    x = x + length;
                }
            } else {
                if (car_gap < 0) {
                    x = x + gap + length;
                } else {
                    x = x + car_gap + length;
                }
            }
            car_count += 1;
        }
    }

    if (this->car_list.size() > 2) {
        for (size_t i = 1; i < this->car_list.size() - 1; ++i) {
            this->car_list[i]->leader = this->car_list[i + 1];
            this->car_list[i]->follower = this->car_list[i - 1];
        }
        this->car_list.front()->leader = this->car_list[1];
        this->car_list.back()->follower = this->car_list[this->car_list.size() - 2];
    } else {
        this->car_list.front()->leader = this->car_list.back();
        this->car_list.back()->follower = this->car_list.front();
    }

    if (this->is_circle) {
        this->car_list.front()->follower = this->car_list.back();
        this->car_list.back()->leader = this->car_list.front();
    }

    std::vector<int> result;
    result.reserve(this->car_list.size());
    for (const auto& car : this->car_list) {
        result.push_back(car->ID);
    }

    return result;
}

void LaneAbstract::run_config(
        bool data_save_,
        UpdateMethod update_method_,
        int warm_up_step_,
        double dt_,
        int sim_step_,
        bool yield,
        bool force_speed_limit_
        ) {
    this->data_save = data_save_;
    this->warm_up_step = warm_up_step_;
    this->dt = dt_;
    this->sim_step = sim_step_;
    this->yield_ = yield;
    this->force_speed_limit = force_speed_limit_;
    this->state_update_method = update_method_;
}

void LaneAbstract::run() {
    // 整个仿真能够运行sim_step的仿真步
    if (!road_control) {
        while (this->sim_step != this->step_) {
            run_first_part();
            run_second_part();
            run_third_part();
        }
    } else {
        throw std::runtime_error("road控制请将road_control设置为true！");
    }
}

/**
 * 1. 车辆生成
 * 2. 数据记录
 * 3. 计算跟驰结果
 */
void LaneAbstract::run_first_part() {
    if (!this->is_circle) {
        this->car_summon();
    }
    // 能够记录warm_up_step仿真步时的车辆数据
    if (this->data_save && this->step_ >= this->warm_up_step) {
        this->record();
    }
    this->step(); // 未更新状态，但已经计算跟驰结果
}

/**
 * 根据计算的跟驰结果，更新车辆状态
 */
void LaneAbstract::run_second_part() {
    // 控制车辆对应的step需要在下一个仿真步才能显现到数据记录中
    this->update_state(); // 更新车辆状态
}

/**
 * 更新仿真步计数器
 */
void LaneAbstract::run_third_part() {
    this->step_ += 1;
    this->time_ += this->dt;
}

void LaneAbstract::car_state_update_common(Vehicle* car) {
    double car_speed_before = car->v;

    if (this->state_update_method == UpdateMethod::Ballistic || this->state_update_method == UpdateMethod::Euler) {
        car->v += car->cf_acc * this->dt;
    } else if (this->state_update_method == UpdateMethod::Trapezoidal) {
        car->v += (car->cf_acc + car->a) * this->dt / 2;
    } else {
        throw std::runtime_error("更新方式未实现！");
    }

    if (this->force_speed_limit && car->v > this->get_speed_limit(car->x, car->type)) {
        double speed_limit_ = this->get_speed_limit(car->x, car->type);
        car->a = (speed_limit_ - car_speed_before) / this->dt;
        car->v = speed_limit_;
    }

    if (car->v > car->cf_model->get_expect_speed()) {
        double expect_speed = car->cf_model->get_expect_speed();
        car->a = (expect_speed - car_speed_before) / this->dt;
        car->v = expect_speed;
    } else if (car->v < 0) {
        throw std::runtime_error("车辆速度出现负数！");
    } else {
        car->a = car->cf_acc;
    }

    if (this->state_update_method == UpdateMethod::Ballistic) {
        car->x += (car_speed_before + car->v) * this->dt / 2;
    } else if (this->state_update_method == UpdateMethod::Euler) {
        car->x += car->v * this->dt;
    } else if (this->state_update_method == UpdateMethod::Trapezoidal) {
        car->x += car_speed_before * this->dt + car->a * std::pow(this->dt, 2) / 2;
    } else {
        throw std::runtime_error("更新方式未实现！");
    }
}

void LaneAbstract::record() {
    for (const auto& car : this->car_list) {
        if (car->type != VType::OBSTACLE) {
            car->record();
        }
    }
}

Vehicle * LaneAbstract::make_dummy_car(double pos) {
    auto* car = new Vehicle{this, VType::OBSTACLE, -1, 1e-5};
    car->set_cf_model(CFM::DUMMY, std::map<std::string, double> {});
    car->x = pos + 1e-5;
    car->v = 0;
    car->a = 0;
    return car;
}

void LaneAbstract::set_block(double pos) {
    Vehicle * dummy_car = make_dummy_car(pos);
    this->car_insert_by_instance(dummy_car);
}

int LaneAbstract::get_appropriate_car() const {
    double pos = this->lane_length / 2;
    std::vector<double> car_pos;
    car_pos.reserve(this->car_list.size());
    for (const auto& car : this->car_list) {
        car_pos.push_back(car->x);
    }

    auto pos_ = std::remove_if(car_pos.begin(), car_pos.end(), [pos](float x) { return x >= pos; });
    car_pos.erase(pos_, car_pos.end());

    auto max_pos_it = std::max_element(car_pos.begin(), car_pos.end());
    int max_pos_index = static_cast<int>(std::distance(car_pos.begin(), max_pos_it));

    return this->car_list[max_pos_index]->ID;
}

void LaneAbstract::take_over(int car_id, double acc_values) {
    for (auto& car : this->car_list) {
        if (car->ID == car_id) {
            car->cf_acc = acc_values;
            break; // Assuming each car ID is unique, stop the loop once we find the matching car.
        }
    }
}

int LaneAbstract::car_insert(double car_length, VType car_type, double car_pos, double car_speed, double car_acc,
                             CFM cf_name, const std::map<std::string, double>& cf_param, const std::map<std::string, double>& car_param,
                             LCM lc_name, const std::map<std::string, double>& lc_param) {
    Vehicle* car = make_car(car_length, car_type, car_pos, car_speed, car_acc, cf_name, cf_param, car_param, lc_name, lc_param);
    this->car_insert_by_instance(car);
    return car->ID;
}

void LaneAbstract::car_remove(Vehicle* car, bool put_out_car_has_data) {
    this->car_list.erase(std::remove(this->car_list.begin(), this->car_list.end(), car), this->car_list.end());
    if (car->leader != nullptr) {
        car->leader->follower = car->follower;
    }
    if (car->follower != nullptr) {
        car->follower->leader = car->leader;
    }

    if (put_out_car_has_data) {
        this->out_car_has_data.push_back(car);
    } else {
        delete car;
    }
}

Vehicle* LaneAbstract::make_car(double car_length, VType car_type, double car_pos, double car_speed, double car_acc,
                                CFM cf_name, const std::map<std::string, double>& cf_param, const std::map<std::string, double>& car_param,
                                LCM lc_name, const std::map<std::string, double>& lc_param) {
    auto* car = new Vehicle{this, car_type, this->get_new_car_id(), car_length};
    car->set_cf_model(cf_name, cf_param);
    car->set_lc_model(lc_name, lc_param); // Use an empty string if lc_name is not provided
    car->set_car_param(const_cast<std::map<std::string, double> &>(car_param));
    car->x = car_pos;
    car->v = car_speed;
    car->a = car_acc;
    return car;
}

bool LaneAbstract::car_insert_by_instance(Vehicle * car, bool is_dummy) {
    car->lane = this;
    car->leader = car->follower = nullptr;
    if (!this->car_list.empty()) {
        std::vector<double> pos_list;
        pos_list.reserve(this->car_list.size());
        for (const auto& lane_car : this->car_list) {
            pos_list.push_back(lane_car->x);
        }

        auto it = std::upper_bound(pos_list.begin(), pos_list.end(), car->x);
        long long int index_ = std::distance(pos_list.begin(), it);

        Vehicle* follower;
        if (index_ != 0) {
            follower = this->car_list[index_ - 1];
        } else {
            if (this->is_circle) {
                follower = this->car_list.back();
            } else {
                follower = nullptr;
            }
        }

        if (follower != nullptr) {
            Vehicle* leader = follower->leader;

            follower->leader = car;
            car->follower = follower;
            car->leader = leader;
            if (leader != nullptr) {
                leader->follower = car;
            }
        } else {
            car->leader = this->car_list.front();
            this->car_list.front()->follower = car;
        }

        if (!is_dummy) {
            this->car_list.insert(this->car_list.begin() + index_, car);
        }
    } else {
        if (is_dummy) {
            throw std::runtime_error("请在初始化正常车辆后设置dummy类型的车辆！");
        }
        this->car_list.push_back(car);
        if (this->is_circle) {
            car->leader = car;
            car->follower = car;
        }
    }
    return true;
}

double LaneAbstract::get_car_info(int id_, C_Info info_name) const {
    for (auto car : this->car_list) {
        if (car->ID == id_) {
            if (info_name == C_Info::x) {
                return car->x;
            }
            if (info_name == C_Info::v) {
                return car->v;
            }
            if (info_name == C_Info::a) {
                return car->a;
            }
            if (info_name == C_Info::gap) {
                return car->gap();
            }
            if (info_name == C_Info::dhw) {
                return car->dhw();
            }
            throw std::runtime_error("C_Info未创建！");
        }
    }
    throw std::runtime_error(std::string("未找到！"));
}

Vehicle* LaneAbstract::get_car(int id_) {
    for (auto car : this->car_list) {
        if (car->ID == id_) {
            return car;
        }
    }
    return nullptr;
}

int LaneAbstract::car_insert_middle(double car_length, VType car_type, double car_speed, double car_acc,
                                    CFM cf_name, std::map<std::string, double>& cf_param,
                                    std::map<std::string, double>& car_param,
                                    int front_car_id,
                                    LCM lc_name, const std::map<std::string, double>& lc_param) {
    int follower_id = get_relative_id(front_car_id, -1);
    double follower_pos = get_car_info(follower_id, C_Info::x);
    double follower_dhw = get_car_info(follower_id, C_Info::dhw);
    double front_length = get_car(front_car_id)->length;

    if (follower_dhw / 2 < car_length || follower_dhw / 2 < front_length) {
        std::cout << "空间不足，插入失败！" << std::endl;
        return -1;
    }

    double pos = follower_pos + follower_dhw / 2;
    Vehicle* car = make_car(car_length, car_type, pos, car_speed, car_acc, cf_name, cf_param, car_param, lc_name, lc_param);
    car_insert_by_instance(car);
    return car->ID;
}

int LaneAbstract::get_relative_id(int id_, int offset) {
    Vehicle* car = get_relative_car_by_id(id_, offset);
    if (car)
        return car->ID;
    return -1;
}

Vehicle* LaneAbstract::get_relative_car_by_id(int id_, int offset) {
    Vehicle* car = nullptr;
    for (auto c : car_list) {
        if (c->ID == id_) {
            while (offset != 0) {
                if (offset > 0) {
                    if (c->leader != nullptr)
                        car = c->leader;
                    offset -= 1;
                } else {
                    if (c->follower != nullptr)
                        car = c->follower;
                    offset += 1;
                }
            }
            return car;
        }
    }
    return nullptr;
}

std::pair<Vehicle*, Vehicle*> LaneAbstract::get_relative_car(double pos) {
    for (auto car : car_list) {
        if (car->x > pos) {
            return std::make_pair(car->follower, car);
        }
    }

    if (!car_list.empty()) {
        if (is_circle) {
            return std::make_pair(car_list.back(), car_list.front());
        } else {
            return std::make_pair(car_list.back(), nullptr);
        }
    }
    return std::make_pair(nullptr, nullptr);
}

void LaneAbstract::car_param_update(int id_,
                                    std::map<std::string, double>& cf_param,
                                    std::map<std::string, double>& lc_param,
                                    std::map<std::string, double>& car_param) {
    Vehicle* car = get_car(id_);
    if (car != nullptr) {
        car->cf_model->param_update(cf_param);
        car->lc_model->param_update(lc_param);
        car->set_car_param(car_param);
    }
}

LaneAbstract::~LaneAbstract() {
    for (auto* car : car_list) {
        delete car;
    }
    for (auto* car : dummy_car_list) {
        delete car;
    }
}
