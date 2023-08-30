//
// Created by yzbyx on 2023/7/21.
//

#ifndef TRASIM_ROAD_H
#define TRASIM_ROAD_H


#include <vector>
#include <map>
#include "../../Constants.h"
#include "LaneAbstract.h"
#include "../../../ui/UI.h"

class Vehicle;

class UI;

class DataProcessor;

struct VehicleData;

class Road {
public:
    explicit Road(double length = 1000, int id = 0);

    ~Road();

    int get_new_car_id();

    int ID;
    double lane_length;
    std::vector<LaneAbstract *> lane_list;
    int id_accumulate;

    int step_;
    int sim_step;
    double dt;
    double time_;

    bool has_ui;
    int frame_rate;
    UI *ui;

    std::map<C_Info, std::vector<double>> road_total_data;
    DataProcessor *data_processor;

    /**
     * 需要一次性完成配置
     *
     * @param lane_num
     * @param is_circle
     * @param real_index
     * @return
     */
    std::vector<LaneAbstract *> add_lanes(int lane_num, bool is_circle, const std::vector<int> &real_index);

    static void step_lane_change(LaneAbstract *lane);

    /**
     * 检查以及修正换道模型提供的位置
     * @param target_lane
     * @param car_lc_last 同一个车道循环中，上一个完成换道的车辆
     * @param car
     * @return 是否满足换道条件（取决于是否之前有车换到了同一个空档内）
     */
    static bool check_and_correct_lc_pos(LaneAbstract *target_lane, Vehicle *car_lc_last, Vehicle *car);

    static std::tuple<LaneAbstract *, LaneAbstract *, std::vector<SECTION_TYPE>>
    get_available_adjacent_lane(LaneAbstract *lane, double pos, VType car_type);

    double get_car_info(int car_id, C_Info info, int lane_add_num = -1);

    int car_insert_middle(int lane_add_num, int front_car_id, VehicleData &data);

    std::vector<int> find_on_lanes(int car_id);

    void cf_take_over(int car_id, double acc_values);

    int get_appropriate_car(int lane_add_num);

    std::vector<LaneAbstract::LaneIterator> lanes_iter;

    // 迭代器部分
    class RoadIterator {
    private:
        Road *ptr;
        int stage;
    public:
        explicit RoadIterator(Road *p);

        std::pair<int, int> operator*();

        RoadIterator &operator++();

        bool operator!=(const RoadIterator &other) const;
    };

    // 返回迭代器的起始位置
    RoadIterator begin() {
        return RoadIterator(this);
    }

    // 返回迭代器的结束位置
    RoadIterator end() {
        return RoadIterator(this);
    }

    /**
     * 完成车辆换道模型计算以及更新，更新方式从内侧到外侧车道，从下游到上游车辆
     *
     * 调用此函数之前，可以使用lc_take_over函数控制车辆换道模型的输出结果
     */
    void run_second_part();

    /**
     * 完成车辆跟驰模型的计算以及更新，更新方式从下游到上游车辆
     *
     * 调用此函数之前，可以使用cf_take_over函数控制车辆换道模型的输出结果
     */
    void run_first_part();

    void run(bool data_save, bool has_ui, double dt_, int sim_step_, int frame_rate, int warm_up_step);

    void run_config(bool data_save, bool has_ui, double dt_, int sim_step_, int frame_rate, int warm_up_step);

    std::map<C_Info, std::vector<double>> &get_road_total_data();

    void lc_take_over(int car_id, const std::tuple<LaneAbstract *, double, double, double> &lc_result);

    /**
     * 将换道模型结果更新到场景
     */
    static void update_lc_state(LaneAbstract *lane);

    int get_car_num_on_road();
};

#endif //TRASIM_ROAD_H
