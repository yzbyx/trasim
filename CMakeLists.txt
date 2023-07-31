cmake_minimum_required(VERSION 3.25)
project(trasim)

set(CMAKE_CXX_STANDARD 17)

set(SFML_DIR "lib/SFML/lib/cmake/SFML")

add_executable(trasim test/trasim_test/road_test.cpp core/Vehicle.cpp core/kinematics/cfm/CFM_Dummy.cpp core/kinematics/cfm/CFM_Dummy.h core/kinematics/cfm/CFModel.h core/kinematics/Model.h core/kinematics/cfm/CFM_IDM.cpp core/kinematics/cfm/CFM_IDM.h core/kinematics/Model.cpp core/frame/micro/LaneAbstract.cpp core/frame/micro/LaneAbstract.h core/kinematics/lcm/LCModel.cpp core/kinematics/lcm/LCModel.h core/Constants.h core/Obstacle.cpp core/kinematics/cfm/CFModel.cpp core/frame/micro/Road.cpp core/frame/micro/Road.h core/data/DataContainer.cpp core/data/DataContainer.h core/data/DataProcessor.cpp core/data/DataProcessor.h core/kinematics/lcm/LCModel_KK.cpp core/kinematics/lcm/LCModel_KK.h test/cpp_test/base.cpp test/cpp_test/base.h test/cpp_test/child.cpp test/cpp_test/child.h core/frame/micro/LaneCircle.cpp core/frame/micro/LaneCircle.h core/frame/micro/LaneOpen.cpp core/frame/micro/LaneOpen.h util/timer.cpp util/timer.h util/save.cpp util/save.h ui/UI.cpp ui/UI.h)

find_package(SFML COMPONENTS graphics window system)

if (NOT SFML_FOUND)
    message(FATAL_ERROR "SFML NOT FOUND!")
else()
    message("SFML FOUND!")
endif()

#SET(CMAKE_BUILD_TYPE "Release")
# 将SFML的头文件路径添加到包含路径
target_include_directories(trasim PRIVATE ${SFML_INCLUDE_DIR})

# 将SFML的库路径添加到链接路径
target_link_directories(trasim PRIVATE ${SFML_LIBRARY_DIRS})

# 将SFML的链接库添加到可执行文件
target_link_libraries(trasim PRIVATE sfml-system sfml-window sfml-graphics)
