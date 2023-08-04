//
// Created by yzbyx on 2023/7/20.
//

#ifndef TRASIM_CONSTANTS_H
#define TRASIM_CONSTANTS_H

#include <vector>
#include <random>
#include <map>

namespace RANDOM {
    static std::uniform_real_distribution<double> DIS12(0, 1);
    static std::mt19937 RND = std::mt19937(0);
    static std::mt19937 CFM_RND = std::mt19937(1);
    static std::mt19937 LCM_RND = std::mt19937(2);
}

enum class CFM{
    IDM,
    DUMMY,
    NONE
};

const std::vector<CFM> ALL_CFM = {
        CFM::IDM,
        CFM::DUMMY,
        CFM::NONE
};

enum class LCM {
    /**
     * KK模型的换道策略
     */
    KK,
    /**
     * Kerner对自动驾驶类车辆的换道规则
     */
     ACC,
     NONE
};

const std::vector<LCM> ALL_LCM = {
        LCM::KK,
        LCM::ACC,
        LCM::NONE
};

enum class C_Info {
    lane_add_num,
    id,
    step,
    time,
    a,
    v,
    x,
    dhw,
    thw,
    gap,
    dv,
    cf_id,
    lc_id,
    car_type,
    safe_ttc,
    safe_tet,
    safe_tit,
    safe_picud,
    safe_picud_KK
};

const std::vector<C_Info> ALL_C_INFO = {
    C_Info::lane_add_num,
    C_Info::id,
    C_Info::step,
    C_Info::time,
    C_Info::a,
    C_Info::v,
    C_Info::x,
    C_Info::dhw,
    C_Info::thw,
    C_Info::gap,
    C_Info::dv,
    C_Info::cf_id,
    C_Info::lc_id,
    C_Info::car_type,
    C_Info::safe_ttc,
    C_Info::safe_tet,
    C_Info::safe_tit,
    C_Info::safe_picud,
    C_Info::safe_picud_KK
};

const std::map<C_Info, std::string> ALL_C_INFO_2_STRING = {
        {C_Info::lane_add_num , "Lane Add Num"},
        {C_Info::id, "ID"},
        {C_Info::step, "Step"},
        {C_Info::time, "Time [s]"},
        {C_Info::a, "Acceleration [m/s^2]"},
        {C_Info::v, "Velocity [m/s]"},
        {C_Info::x, "Position [m]"},
        {C_Info::dhw, "Distance Headway [m]"},
        {C_Info::thw, "Time Headway [s]"},
        {C_Info::gap, "Gap [m]"},
        {C_Info::dv, "Dv [m/s]"},
        {C_Info::cf_id, "cf model ID"},
        {C_Info::lc_id, "lc model ID"},
        {C_Info::car_type, "car type"},
        {C_Info::safe_ttc, "ttc (s)"},
        {C_Info::safe_tet, "tet"},
        {C_Info::safe_tit, "tit (s)"},
        {C_Info::safe_picud, "picud (m)"},
        {C_Info::safe_picud_KK, "picud_KK (m)"}
};

enum class Color {
    BLUE,
    GREEN,
    RED
};

static const std::vector<Color> ALL_COLOR = {
      Color::RED,
      Color::GREEN,
      Color::BLUE
};

static const std::map<Color, std::vector<int>> COLOR_2_RGB = {
        {Color::BLUE, {0, 0, 255}},
        {Color::GREEN, {0, 255, 0}},
        {Color::RED, {255, 0, 0}}
};

enum class SECTION_TYPE {
    /**
     * 基本路段
     */
    BASE = 0,
    /**
     * 入口匝道区域
     */
    ON_RAMP,
    /**
     * 出口匝道区域
     */
    OFF_RAMP,
    /**
     * 禁止向左换道
     */
    NO_LEFT,
    /**
     * 禁止向右换道
     */
    NO_RIGHT,
    /**
     * 禁止左侧车辆换入
     */
    NO_LEFT_CAR,
    /**
     * 禁止右侧车辆换入
     */
    NO_RIGHT_CAR
};

static const std::vector<SECTION_TYPE> ALL_SECTION_TYPE = {
        SECTION_TYPE::BASE,
        SECTION_TYPE::ON_RAMP,
        SECTION_TYPE::OFF_RAMP,
        SECTION_TYPE::NO_LEFT,
        SECTION_TYPE::NO_RIGHT,
        SECTION_TYPE::NO_LEFT_CAR,
        SECTION_TYPE::NO_RIGHT_CAR
};

enum class VType {
    PASSENGER = 0,
    TRUCK,
    BUS,
    OBSTACLE
};

const std::vector<VType> ALL_V_TYPE = {
        VType::PASSENGER,
        VType::TRUCK,
        VType::BUS,
        VType::OBSTACLE
};

enum class UpdateMethod {
    Euler,
    Ballistic,
    Trapezoidal
};

enum class THW_DISTRIBUTION {
    Uniform,
    Exponential,
};
#endif //TRASIM_CONSTANTS_H
