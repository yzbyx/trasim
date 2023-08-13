cmake_minimum_required(VERSION 3.22.1)
project(trasim)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -g")

IF (WIN32)
    MESSAGE(STATUS "Now is windows")
    set(SFML_DIR "lib/SFML/lib/cmake/SFML")
ELSEIF (APPLE)
    MESSAGE(STATUS "Now is Apple systens.")
ELSEIF (UNIX)
    MESSAGE(STATUS "Now is UNIX-like OS's.")
#    set(SFML_DIR "lib/SFML-2.6.0-linux/lib/cmake/SFML")
ENDIF ()

add_executable(trasim test/trasim_test/road_test.h core/Vehicle.cpp core/kinematics/cfm/CFM_Dummy.cpp core/kinematics/cfm/CFM_Dummy.h core/kinematics/cfm/CFModel.h core/kinematics/Model.h core/kinematics/cfm/CFM_IDM.cpp core/kinematics/cfm/CFM_IDM.h core/kinematics/Model.cpp core/frame/micro/LaneAbstract.cpp core/frame/micro/LaneAbstract.h core/kinematics/lcm/LCModel.cpp core/kinematics/lcm/LCModel.h core/Constants.h core/Obstacle.cpp core/kinematics/cfm/CFModel.cpp core/frame/micro/Road.cpp core/frame/micro/Road.h core/data/DataContainer.cpp core/data/DataContainer.h core/data/DataProcessor.cpp core/data/DataProcessor.h core/kinematics/lcm/LCModel_KK.cpp core/kinematics/lcm/LCModel_KK.h test/cpp_test/base.cpp test/cpp_test/base.h test/cpp_test/child.cpp test/cpp_test/child.h core/frame/micro/LaneCircle.cpp core/frame/micro/LaneCircle.h core/frame/micro/LaneOpen.cpp core/frame/micro/LaneOpen.h util/timer.cpp util/timer.h util/saver.cpp util/saver.h ui/UI.cpp ui/UI.h
        util/print.cpp
        util/print.h
        test/trasim_test/lane_test.h
        test/trasim_test/test.cpp
        test/trasim_test/test.cpp)

find_package(SFML COMPONENTS graphics window system audio)
if (NOT SFML_FOUND)
    message(FATAL_ERROR "SFML NOT FOUND!")
else()
    message(STATUS "SFML FOUND!")
endif()
# 将SFML的头文件路径添加到包含路径
target_include_directories(trasim PRIVATE ${SFML_INCLUDE_DIR})
# 将SFML的库路径添加到链接路径
target_link_directories(trasim PRIVATE ${SFML_LIBRARY_DIRS})
# 将SFML的链接库添加到可执行文件
target_link_libraries(trasim PRIVATE sfml-system sfml-window sfml-graphics sfml-audio)

# Find Python
find_package(Python REQUIRED COMPONENTS Development)
if (NOT Python_FOUND)
    message(FATAL_ERROR "Python NOT FOUND!")
else()
    message(STATUS "Python FOUND!")
endif()
# Include Python headers
target_include_directories(trasim PRIVATE ${Python_INCLUDES})
# Link Python libraries
target_link_libraries(trasim PRIVATE ${Python_LIBRARIES})

#SET(CMAKE_BUILD_TYPE "Release")

