//
// Created by yzbyx on 2023/7/20.
//

#ifndef TRASIM_LANEABSTRACT_H
#define TRASIM_LANEABSTRACT_H

#include <vector>
#include <map>
#include <string>
#include <set>
#include "Road.h"
#include "../../Constants.h"
#include "../../Vehicle.h"
#include "../../data/DataContainer.h"
#include "../../data/DataProcessor.h"

class Vehicle;

class LaneAbstract {
public:
    int ID;
    int index;
    int add_num;
    Road* road{};
    std::vector<LaneAbstract*> left_neighbour_lanes;
    std::vector<LaneAbstract*> right_neighbour_lanes;

    float default_speed_limit;
    int car_num_total;
    bool is_circle;
    double lane_length;
    std::map<VType, std::map<SECTION_TYPE, std::array<double, 2>>> section_type;
    std::map<VType, std::map<float, std::array<double, 2>>> speed_limit;

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

    std::vector<Vehicle*> car_list;
    std::vector<Vehicle*> dummy_car_list;
    std::vector<Vehicle*> out_car_has_data;

    int step_;
    double time_;
    bool yield_;
    bool road_control;
    bool force_speed_limit;
    UpdateMethod state_update_method;

    double dt;
    int warm_up_step;
    int sim_step;

    bool data_save;
    DataContainer* data_container{};
    DataProcessor* data_processor{};

    explicit LaneAbstract(float lane_length = 1000, float speed_limit = 30);
    ~LaneAbstract();

    int get_new_car_id();
    void set_section_type(SECTION_TYPE type_, double start_pos = -1, double end_pos = -1,
                          std::vector<VType> car_types = {});
    std::set<SECTION_TYPE> get_section_type(double pos, VType car_type);

    void set_speed_limit(double speed_limit = 30, double start_pos = -1, double end_pos = -1, std::vector<VType> car_types = {});

    double get_speed_limit(double pos, VType car_type);

    [[nodiscard]] int car_num() const;

    void car_config(float car_num, float car_length, VType car_type, float car_initial_speed, bool speed_with_random,
                    CFM cf_name, const std::map<std::string, double>& cf_param, const std::map<std::string, double>& car_param,
                    LCM lc_name = LCM::NONE, const std::map<std::string, double>& lc_param = {});

    std::vector<int> car_load(float car_gap = -1, int jam_num = -1);

    void run_config(bool data_save_ = true,
             UpdateMethod update_method_ = UpdateMethod::Euler,
             int warm_up_step_ = 3000,
             double dt_ = 0.1,
             int sim_step_ = 6000,
             bool yield = true,
             bool force_speed_limit_ = false);

    void run_first_part();

    void run();

    void run_second_part();

    void run_third_part();

    void car_state_update_common(Vehicle* car);

    virtual void update_state() = 0;
    virtual void step() = 0;
    virtual void car_summon() = 0;

    void record();
    void set_block(double pos);

    int car_insert(double car_length, VType car_type, double car_pos, double car_speed, double car_acc, CFM cf_name,
                   const std::map<std::string, double>& cf_param, const std::map<std::string, double>& car_param, LCM lc_name,
                   const std::map<std::string, double>& lc_param);

    void take_over(int car_id, double acc_values);

    [[nodiscard]] int get_appropriate_car() const;

    std::pair<Vehicle *, Vehicle *> get_relative_car(double pos);

protected:
    Vehicle * make_dummy_car(double pos);

    Vehicle* make_car(double car_length, VType car_type, double car_pos, double car_speed, double car_acc, CFM cf_name,
                     const std::map<std::string, double> &cf_param, const std::map<std::string, double> &car_param,
                     LCM lc_name, const std::map<std::string, double> &lc_param);

    void car_remove(Vehicle *car, bool put_out_car_has_data);

    bool car_insert_by_instance(Vehicle *car, bool is_dummy = false);

    [[nodiscard]] double get_car_info(int id_, C_Info info_name) const;

    Vehicle *get_car(int id_);

    int car_insert_middle(double car_length, VType car_type, double car_speed, double car_acc, CFM cf_name,
                          std::map<std::string, double> &cf_param, std::map<std::string, double> &car_param,
                          int front_car_id, LCM lc_name, const std::map<std::string, double> &lc_param);

    int get_relative_id(int id_, int offset);

    Vehicle *get_relative_car_by_id(int id_, int offset);

    void car_param_update(int id_, std::map<std::string, double> &cf_param, std::map<std::string, double> &lc_param,
                          std::map<std::string, double> &car_param);
};

#endif //TRASIM_LANEABSTRACT_H
