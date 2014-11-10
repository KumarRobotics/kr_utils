cmake_minimum_required(VERSION 2.8.3)
project(kr_common)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11 -Wall")

find_package(catkin REQUIRED COMPONENTS
    roscpp roslib
    )

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp
    )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )

add_executable(test_timer test/test_timer.cpp)
add_executable(test_pretty_print test/test_pretty_print.cpp)
add_executable(test_pretty_printer test/test_pretty_printer.cpp)
add_executable(test_profiler test/test_profiler.cpp)
add_executable(test_sample test/test_sample.cpp)
add_executable(test_url_parser test/test_url_parser.cpp)
target_link_libraries(test_url_parser ${catkin_LIBRARIES})


if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(gtest_kr_common
    test/gtest_kr_common.cpp
    test/test_ros_helper.cpp
    )
  target_link_libraries(gtest_kr_common
    ${catkin_LIBRARIES}
    ${GTEST_LIBRARIES}
    )
endif()

