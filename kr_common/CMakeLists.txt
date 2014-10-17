cmake_minimum_required(VERSION 2.8.3)
project(kr_common)

add_definitions(-std=c++11 -Wall)

find_package(catkin REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    #  LIBRARIES kr_common
    #  CATKIN_DEPENDS other_catkin_pkg
    #  DEPENDS system_lib
    )

include_directories(include)
add_executable(test_timer test/test_timer.cpp)

