cmake_minimum_required(VERSION 2.8.3)
project(rviz_helper)

add_definitions(-Wall -std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs geometry_msgs nav_msgs
    visualization_msgs
    tf2_ros
    )

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES rviz_helper
    CATKIN_DEPENDS std_msgs geometry_msgs nav_msgs roscpp visualization_msgs tf2_ros
    DEPENDS system_lib
    )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )

add_library(${PROJECT_NAME}
    src/visualizer.cpp
    )
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    )

add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
    )
