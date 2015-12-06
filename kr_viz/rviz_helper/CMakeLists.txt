cmake_minimum_required(VERSION 2.8.3)
project(rviz_helper)

add_definitions(-Wall -std=c++11)

find_package(cmake_modules)
find_package(Eigen REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp tf2_ros
    std_msgs geometry_msgs nav_msgs visualization_msgs
    )

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES rviz_helper
    CATKIN_DEPENDS std_msgs geometry_msgs nav_msgs roscpp visualization_msgs tf2_ros
    DEPENDS Eigen
    )

include_directories(
    include
    ${EIGEN_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    )

add_library(${PROJECT_NAME}
    src/marker_visualizer.cpp
    src/kr_viz.cpp
    src/marker_covariance.cpp
    )
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    )

add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
    )

add_executable(sample_viz samples/sample_viz.cpp)
target_link_libraries(sample_viz
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    )
