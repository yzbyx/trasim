//
// Created by yzbyx on 2023/7/20.
//

#ifndef TRASIM_LANEABSTRACT_H
#define TRASIM_LANEABSTRACT_H

#include <vector>
#include <map>
#include <string>
#include <set>
#include "../../Constants.h"
#include "../../Vehicle.h"
#include "../../data/DataContainer.h"
#include "../../data/DataProcessor.h"

class Vehicle;

class Road;

class VehicleData;

class LaneAbstract: public std::enable_shared_from_this<LaneAbstract> {
public:
    std::string ID;
    /**
     * 指定的车道编号，从内向外依次为0,1,2...
     */
    int index;
    /**
     * 车道添加的次序
     */
    int add_num;
    std::shared_ptr<Road> road{};
    std::vector<std::shared_ptr<LaneAbstract>> left_neighbour_lanes;
    std::vector<std::shared_ptr<LaneAbstract>> right_neighbour_lanes;

    double default_speed_limit;
    int car_num_total;
    bool is_circle;
    double lane_length;
    /**
     * 车辆类型，路段类型，类型区间集合
     */
    std::map<VType, std::map<SECTION_TYPE, std::vector<std::pair<double, double>>>> section_type;
    /**
     * 车辆类型，限制区间起点集合，限制区间终点集合，限制区间速度集合
     */
    std::map<VType, std::vector<std::tuple<double, double, double>>> speed_limit;
    /**
     * 车辆类型，控制类型，控制区间集合
     */
    std::map<VType, std::map<CONTROL_TYPE, std::vector<std::pair<double, double>>>> control_type;

    int id_accumulate;
    std::vector<int> car_num_list;
    std::vector<VType> car_type_list;
    std::vector<double> car_length_list;
    std::vector<double> car_initial_speed_list;
    std::vector<bool> speed_with_random_list;
    std::vector<CFM> cf_name_list;
    std::vector<std::map<std::string, double>> cf_param_list;
    std::vector<std::map<std::string, double>> car_param_list;
    std::vector<LCM> lc_name_list;
    std::vector<std::map<std::string, double>> lc_param_list;

    std::vector<std::shared_ptr<Vehicle>> car_list;
    std::vector<std::shared_ptr<Vehicle>> dummy_car_list;
    std::vector<std::shared_ptr<Vehicle>> out_car_has_data;

    int step_;
    double time_;
    bool road_control;
    bool force_speed_limit;
    UpdateMethod state_update_method;

    double dt;
    int warm_up_step;
    int sim_step;

    bool data_save;
    std::shared_ptr<DataContainer> data_container;
    std::shared_ptr<DataProcessor> data_processor{};

    explicit LaneAbstract(double lane_length_ = 1000, double speed_limit_ = 30);

    virtual ~LaneAbstract();

    int get_new_car_id();

    void set_section_type(SECTION_TYPE type_, double start_pos = -1, double end_pos = -1,
                          std::vector<VType> car_types = {});

    void set_control_type(CONTROL_TYPE type_, double start_pos = -1, double end_pos = -1,
                          std::vector<VType> car_types = {});

    std::vector<SECTION_TYPE> get_section_type(double pos, VType car_type);

    void set_speed_limit(double speed_limit = 30, double start_pos = -1, double end_pos = -1,
                         std::vector<VType> car_types = {});

    double get_speed_limit(double pos, VType car_type);

    [[nodiscard]] int car_num() const;

    void car_config(double car_num, double car_length, VType car_type, double car_initial_speed, bool speed_with_random,
                    CFM cf_name, const std::map<std::string, double> &cf_param_,
                    const std::map<std::string, double> &car_param_,
                    LCM lc_name = LCM::NONE, const std::map<std::string, double> &lc_param_ = {});

    std::vector<int> car_load(float car_gap = -1, int jam_num = -1);

    void run_config(bool data_save_, UpdateMethod update_method_, int warm_up_step_, double dt_, int sim_step_,
                    bool force_speed_limit_);

    void run_first_part();

    void run();

    void run_second_part();

    void run_third_part();

    void car_state_update_common(const std::shared_ptr<Vehicle>& car);

    virtual void update_state() = 0;

    virtual void step() = 0;

    virtual void car_summon();

    virtual void
    car_loader(double flow_rate_, THW_DISTRIBUTION thw_distribution_, double offset_time, double offset_pos_);

    void
    record();

    void
    set_block(double pos);

    void
    take_over(int car_id, double acc_values);

    [[nodiscard]] int
    get_appropriate_car() const;

    std::pair<std::shared_ptr<Vehicle>, std::shared_ptr<Vehicle>>
    get_relative_car(double pos);

    void
    car_remove(const std::shared_ptr<Vehicle>& car, bool put_out_car_has_data, bool is_lc);

    bool
    car_insert_by_instance(const std::shared_ptr<Vehicle>& car, bool is_dummy);

    [[nodiscard]] double
    get_car_info(int id_, C_Info info_name) const;

    std::shared_ptr<Vehicle>
    make_dummy_car(double pos);

    std::shared_ptr<Vehicle>
    get_car(int id_);

    int
    get_relative_id(int id_, int offset);

    std::shared_ptr<Vehicle>
    get_relative_car_by_id(int id_, int offset);

    void
    car_param_update(int id_, std::map<std::string, double> &cf_param, std::map<std::string, double> &lc_param,
                     std::map<std::string, double> &car_param);

    std::shared_ptr<Vehicle>
    make_car(VehicleData &data);

    int
    car_insert(VehicleData &data);

    int
    car_insert_middle(int front_car_id, VehicleData &data);

    // 迭代器部分
    class LaneIterator {
    private:
        LaneAbstract *ptr;
        int stage;
    public:
        explicit LaneIterator(LaneAbstract *p);

        std::pair<int, int> operator*();

        LaneIterator &operator++();

        bool operator!=(const LaneIterator &other) const;
    };

    // 返回迭代器的起始位置
    LaneIterator begin() {
        return LaneIterator(this);
    }

    // 返回迭代器的结束位置
    LaneIterator end() {
        return LaneIterator(this);
    }

    [[nodiscard]] std::map<C_Info, std::vector<double>> get_lane_total_data() const;

    /**
     * 获取车道线的控制信息（禁止左转、禁止右转）
     */
    std::vector<CONTROL_TYPE> get_control_type(double pos, VType car_type);

    /**
     * 设置车道线的控制信息（禁止左转、禁止右转）
     */
    void
    set_control_type(SECTION_TYPE type_, double start_pos = -1, double end_pos = -1, std::vector<VType> car_types = {});
};

#endif //TRASIM_LANEABSTRACT_H
