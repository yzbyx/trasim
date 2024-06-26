#include <string>
#include <map>
#include "../../core/frame/micro/Road.h"
#include "../../util/saver.h"
#include "../../util/timer.h"

//
// Created by yzbyx on 2023/7/23.
//
int road_test() {
    auto beforeTime = std::chrono::steady_clock::now();

    std::map<std::string, double> _cf_param = {{"lambda",           0.8},
                                               {"original_acc",     0.0},
                                               {"v_safe_dispersed", 1.0},
                                               {"g_tau",            1.4},
                                               {"kdv",              0.3},
                                               {"v0",               30}};
    std::map<std::string, double> _car_param;
    int take_over_index = -1;
    int follower_index = -1;

    double dt = 0.1;
    int warm_up_step = static_cast<int>(30 / dt);
    int sim_step = warm_up_step + static_cast<int>(120 * 6 / dt);
    int offset_step = static_cast<int>(120 * 2 / dt);

    int road_length = 5000;
    int lane_num = 2;
    double v_length = 7.5;

    Road sim(road_length);
    std::vector<std::shared_ptr<LaneAbstract>> lanes = sim.add_lanes(lane_num, false, {});
    for (int i = 0; i < lane_num; i++) {
        if (i != lane_num - 1) {
            lanes[i]->set_speed_limit(30);
        } else {
            lanes[i]->set_speed_limit(22.2);
        }

        if (i == lane_num - 2) {
            lanes[i]->set_section_type(SECTION_TYPE::BASE);
            lanes[i]->set_control_type(CONTROL_TYPE::NO_RIGHT);
        }
        if (i == lane_num - 1) {
            lanes[i]->set_section_type(SECTION_TYPE::ON_RAMP, 2000, -1);
            lanes[i]->set_control_type(CONTROL_TYPE::NO_LEFT, 0, 2000);
            lanes[i]->set_section_type(SECTION_TYPE::BASE, 0, 2000);
            lanes[i]->set_block(2300);
        } else {
            // lanes[i].car_load();
        }

        lanes[i]->car_config(60, v_length, VType::PASSENGER,
                             lanes[i]->get_speed_limit(0, VType::PASSENGER),
                             false,
                             CFM::IDM, _cf_param, {}, LCM::KK, {});
//        lanes[i]->car_config(60, v_length, VType::PASSENGER,
//                             lanes[i]->get_speed_limit(0, VType::PASSENGER), false,
//                             CFM::IDM, _cf_param, {}, LCM::KK, {});

        lanes[i]->data_container->config({}, true);

        if (i != lane_num - 1) {
            lanes[i]->car_loader(2000, THW_DISTRIBUTION::Uniform, 0, 0);
        } else {
            lanes[i]->car_loader(400, THW_DISTRIBUTION::Uniform, 400, 0);
        }
    }

    sim.run_config(true, false, dt, sim_step, 30, warm_up_step);

    for (auto &&[step, stage]: sim) {
        // 当前step的跟驰计算，只有在下一个step才会更新到车辆状态中
//        std::cout << "step: " << step << ", stage: " << stage << std::endl;
        if (warm_up_step + offset_step == step && stage == 1) {
            take_over_index = sim.get_appropriate_car(0);
            std::cout << "take_over_index: " << take_over_index << " step: " << step << std::endl;
        }
        if (warm_up_step + offset_step <= step && warm_up_step + offset_step + 10 / dt > step && stage == 1) {
            sim.cf_take_over(take_over_index, -1);
        }
    }
    std::cout << "main sim completed" << std::endl;

    auto temp = sim.get_road_total_data();
    std::cout << "data collection completed" << std::endl;

    save_data_to_txt("test_cplusplus.txt", temp);
    std::cout << "data saved" << std::endl;

    auto afterTime = std::chrono::steady_clock::now();
    double duration_second = std::chrono::duration<double>(afterTime - beforeTime).count();
    std::cout << "[run_road] time usage: " << duration_second << "s" << std::endl;

    return 0;
}
