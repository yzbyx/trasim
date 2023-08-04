//
// Created by yzbyx on 2023/8/4.
//

#include "../../util/saver.h"
#include "../../util/timer.h"
#include "../../core/frame/micro/Road.h"


int lane_test() {
    time_t timeIn = time(nullptr);
    std::string timeStart = get_current_time();

    bool is_circle = true;
    double length = 1000;
    int car_num = 100;
    double speed_limit = 30;

    double dt = 0.1;
    int warm_up_step = static_cast<int>(300 / dt);
    int sim_step = warm_up_step + static_cast<int>(30 * 60 / dt);
    int offset_step = static_cast<int>(300 / dt);

    int take_over_index = -1;
    int follower_index = -1;

    Road sim(length);
    std::vector<LaneAbstract *> lanes = sim.add_lanes(1, is_circle, {});

    lanes[0]->car_config(car_num, 7.5, VType::PASSENGER, speed_limit,
                                 false, CFM::IDM, {}, {},
                                 LCM::NONE, {});
    lanes[0]->car_load();
    lanes[0]->data_container->config({}, true);

    sim.run_config(true, false, dt, sim_step, -1, warm_up_step);

    int step, stage;

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
}
