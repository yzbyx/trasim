#include <string>
#include <map>
#include "../../core/frame/micro/Road.h"
#include "../../util/saver.h"
#include "../../util/timer.h"

//
// Created by yzbyx on 2023/7/23.
//
int road_test() {
    time_t timeIn = time(nullptr);
    std::string timeStart = get_current_time();

    std::map<std::string, double> _cf_param = {{"lambda", 0.8}, {"original_acc", 0.0}, {"v_safe_dispersed", 1.0},
                                               {"g_tau", 1.4}, {"kdv", 0.3}, {"v0", 30}};
    std::map<std::string, double> _car_param;
    int take_over_index = -1;
    int follower_index = -1;

    double dt = 0.1;
    int warm_up_step = static_cast<int>(30 / dt);
    int sim_step = warm_up_step + static_cast<int>(30 * 6 / dt);
    int offset_step = static_cast<int>(30 / dt);

    bool is_circle = false;
    int road_length = 5000;
    int lane_num = 2;
    double v_length = 7.5;

    Road sim(road_length);
    std::vector<LaneAbstract *> lanes = sim.add_lanes(lane_num, is_circle, {});
    for (int i = 0; i < lane_num; i++) {
        if (i != lane_num - 1) {
            lanes[i]->set_speed_limit(30);
        } else {
            lanes[i]->set_speed_limit(22.2);
        }

        if (i == lane_num - 2) {
            lanes[i]->set_section_type(SECTION_TYPE::BASE);
            lanes[i]->set_section_type(SECTION_TYPE::NO_RIGHT);
        }
        if (i == lane_num - 1) {
            // TODO: 将每种section type下的范围设置为vector，实现存储多个路段
            lanes[i]->set_section_type(SECTION_TYPE::ON_RAMP, 2000, -1);
            lanes[i]->set_section_type(SECTION_TYPE::NO_LEFT, 0, 2000);
            lanes[i]->set_section_type(SECTION_TYPE::BASE, 0, 5000);
            lanes[i]->set_block(10300);
        } else {
            // lanes[i].car_load();
        }

        lanes[i]->car_config(60, v_length, VType::PASSENGER,
                lanes[i]->get_speed_limit(0, VType::PASSENGER), false,
                            CFM::IDM, _cf_param, {}, LCM::KK, {});
//        lanes[i]->car_config(60, v_length, VType::PASSENGER,
//                             0, false,
//                             CFM::IDM, _cf_param, {}, LCM::KK, {});
        lanes[i]->data_container->config({}, true);

        if (i != lane_num - 1) {
            lanes[i]->car_loader(2000, THW_DISTRIBUTION::Uniform, 0, 0);
        } else {
            lanes[i]->car_loader(400, THW_DISTRIBUTION::Uniform, 400, 0);
        }
    }

    sim.run_config(true, false, dt, sim_step, -1, warm_up_step);

    int step;
    int stage;

    for (auto && road_it : sim) {
        step = road_it.first;
        stage = road_it.second;
        if (warm_up_step + offset_step == step && stage == 1) {
            take_over_index = sim.get_appropriate_car(0);
            std::cout << "take_over_index: " << take_over_index << std::endl;
        }
//        std::cout << "step: " << step << " car_num: " << sim.get_car_num_on_road() << std::endl;
    }
    std::cout << "sim end" << std::endl;

    auto temp = sim.get_road_total_data();
    std::cout << "data get end" << std::endl;

    save_data_to_txt("D:\\test.txt", temp);
    std::cout << "data saved" << std::endl;

    time_t timeOut = time(nullptr);
    std::string log_string = std::string("[road_test] time usage: ") + timeStart + " + " +
                             std::to_string(timeOut - timeIn) + " s";
    std::cout << log_string << std::endl;

    return 0;
}
