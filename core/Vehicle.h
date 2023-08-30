//
// Created by yzbyx on 2023/6/23.
//

#ifndef TRASIM_VEHICLE_H
#define TRASIM_VEHICLE_H

#include <vector>
#include <map>
#include <string>
#include "Obstacle.h"
#include "kinematics/lcm/LCModel.h"
#include "kinematics/cfm/CFModel.h"

class LaneAbstract;

struct VehicleData {
    VType car_type;
    double car_length;
    double car_pos;
    double car_speed;
    double car_acc;
    CFM cf_name;
    std::map<std::string, double> &cf_param;
    LCM lc_name;
    std::map<std::string, double> &lc_param;
    std::map<std::string, double> &car_param;
};

class Vehicle : public Obstacle {
public:
    explicit Vehicle(LaneAbstract *lane_, VType type_, int id_, double length_);

    int ID;
    LaneAbstract* lane;
    /**
     * 是否驶出路外
     */
    bool is_run_out;

    Vehicle* leader;
    Vehicle* follower;

    double ttc_star;
    std::vector<double> lane_id_list;
    std::vector<double> pos_list;
    std::vector<double> speed_list;
    std::vector<double> acc_list;
    std::vector<double> step_list;
    std::vector<double> time_list;
    std::vector<double> dv_list;
    std::vector<double> gap_list;
    std::vector<double> thw_list;
    std::vector<double> dhw_list;
    std::vector<double> ttc_list;
    std::vector<double> tit_list;
    std::vector<double> tet_list;
    std::vector<double> picud_list;
    std::vector<double> picud_KK_list;

    CFModel* cf_model;
    double cf_acc;

    LCModel* lc_model;
    bool is_cf_take_over;
    bool is_lc_take_over;
    std::tuple<LaneAbstract*, double, double, double> lc_result;
    LaneAbstract* lc_target_lane;
    /**
     * 目标车道的前后车
     */
    std::pair<Vehicle*, Vehicle*> lc_f_l;

    std::vector<SECTION_TYPE> section_type;

    int last_step_lc_status();

    void set_cf_model(CFM cf_name, const std::map<std::string, double> &cf_param);

    void set_lc_model(LCM lc_name, const std::map<std::string, double>& lc_param);

    void step(int index);

    void step_lane_change(int index, LaneAbstract *left_lane, LaneAbstract *right_lane,
                          std::vector<SECTION_TYPE> section_type);

    /**
     * 获取pos与车头之间的距离，pos在前则正，在后则负，环形边界则以距离近者决定正负
     * @param pos
     * @return pos-x（对于环形边界经过修正）
     */
    double get_dist(double pos);

    [[nodiscard]] bool is_first() const;

    std::vector<double> get_data_list(C_Info info);

    void record();

    double gap();

    double dv();

    double dhw();

    double thw();

    double ttc();

    double tit();

    double tet();

    double picud();

    double picud_KK();

    [[nodiscard]] bool has_data() const;

    void set_car_param(std::map<std::string, double> &param);
};

#endif //TRASIM_VEHICLE_H
