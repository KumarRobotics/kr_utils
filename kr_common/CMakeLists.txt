cmake_minimum_required(VERSION 2.8.3)
project(kr_common)

add_definitions(-std=c++11 -Wall)

find_package(catkin REQUIRED)

catkin_package(INCLUDE_DIRS include)

include_directories(include)
add_executable(test_timer test/test_timer.cpp)
add_executable(test_pretty_print test/test_pretty_print.cpp)
add_executable(test_pretty_printer test/test_pretty_printer.cpp)

