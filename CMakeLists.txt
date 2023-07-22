cmake_minimum_required(VERSION 3.25)
project(trasim)

set(CMAKE_CXX_STANDARD 17)

#SET(CMAKE_BUILD_TYPE "Release")

add_executable(trasim main.cpp core/Vehicle.cpp core/kinematics/cfm/CFM_Dummy.cpp core/kinematics/cfm/CFM_Dummy.h core/kinematics/cfm/CFModel.h core/kinematics/Model.h core/kinematics/cfm/CFM_IDM.cpp core/kinematics/cfm/CFM_IDM.h core/kinematics/Model.cpp core/frame/micro/LaneAbstract.cpp core/frame/micro/LaneAbstract.h core/kinematics/lcm/LCModel.cpp core/kinematics/lcm/LCModel.h core/Constants.h core/Obstacle.cpp core/kinematics/cfm/CFModel.cpp core/frame/micro/Road.cpp core/frame/micro/Road.h core/data/DataContainer.cpp core/data/DataContainer.h core/data/DataProcessor.cpp core/data/DataProcessor.h core/kinematics/lcm/LCModel_KK.cpp core/kinematics/lcm/LCModel_KK.h test/cpp_test/base.cpp test/cpp_test/base.h test/cpp_test/child.cpp test/cpp_test/child.h)
