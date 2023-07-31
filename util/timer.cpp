//
// Created by yzbyx on 2023/7/24.
//

#include <sstream>
#include "timer.h"

std::string get_current_time() {
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    std::stringstream ss;
    ss << std::put_time(std::localtime(&t), "%Y-%m-%d_%H-%M-%S");
    ss << "." << std::setfill('0') << std::setw(3) << ms.count();

    return ss.str();
}
