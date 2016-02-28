cmake_minimum_required(VERSION 2.8.3)
project(kr_common)

find_package(catkin REQUIRED COMPONENTS
    roscpp roslib
    )

include(cmake/extras.cmake)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp roslib
    CFG_EXTRAS extras.cmake
    )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )

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

