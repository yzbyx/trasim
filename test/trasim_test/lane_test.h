//
// Created by yzbyx on 2023/8/4.
//

#include "../../util/saver.h"
#include "../../util/timer.h"
#include "../../core/frame/micro/Road.h"

int lane_test() {
    auto beforeTime = std::chrono::steady_clock::now();

    bool is_circle = false;
    double length = 10000;
    int car_num = 50;
    double speed_limit = 30;

    double dt = 0.1;
    int warm_up_step = static_cast<int>(1800 / dt);
    int sim_step = warm_up_step + static_cast<int>(3600 / dt);
    int offset_step = static_cast<int>(500 / dt) + warm_up_step;
    int dec_step = static_cast<int>(50 / dt) + offset_step;
    int maintain_step = static_cast<int>(100 / dt) + dec_step;
    int acc_step = static_cast<int>(50 / dt) + maintain_step;

    int take_over_index = -1;
    int follower_index = -1;

    auto sim = std::make_shared<Road>(length);
    std::vector<std::shared_ptr<LaneAbstract>> lanes = sim->add_lanes(1, is_circle, {});

    lanes[0]->car_config(car_num, 7.5, VType::PASSENGER, speed_limit,
                                 false, CFM::IDM, {}, {},
                                 LCM::NONE, {});
    lanes[0]->car_loader(2000, THW_DISTRIBUTION::Uniform, 0, 0);
    lanes[0]->data_container->add_basic_info();

    sim->run_config(true, false, dt, sim_step, -1, warm_up_step);

    int step, stage;

    for (auto && road_it : *sim) {
        step = road_it.first;
        stage = road_it.second;
        if (step == offset_step && stage == 1) {
            take_over_index = sim->get_appropriate_car(0);
            std::cout << "take_over_index: " << take_over_index << std::endl;
        }
        if (offset_step < step && step < dec_step && stage == 1) {
            sim->cf_take_over(take_over_index, -1);
        }
        if (dec_step < step && step < maintain_step && stage == 1) {
            sim->cf_take_over(take_over_index, 0);
        }
        if (maintain_step < step && step < acc_step && stage == 1) {
            sim->cf_take_over(take_over_index, 1);
        }
        if (step == acc_step && stage == 1) {
            sim->cf_take_over(take_over_index, 0);
        }

//        std::cout << "step: " << step << " car_num: " << sim.get_car_num_on_road() << std::endl;
    }
    std::cout << "sim end" << std::endl;

    auto temp = sim->get_road_total_data();
    std::cout << "data get end" << std::endl;

    save_data_to_txt("test.txt", temp);
    std::cout << "data saved" << std::endl;

    auto afterTime = std::chrono::steady_clock::now();
    double duration_second = std::chrono::duration<double>(afterTime - beforeTime).count();
    std::cout << "[run_lane] time usage: " << duration_second << "s" << std::endl;

    return 0;
}
