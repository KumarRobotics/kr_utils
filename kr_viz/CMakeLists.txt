cmake_minimum_required(VERSION 2.8.3)
project(kr_viz)

find_package(catkin REQUIRED COMPONENTS
    kr_cmake_modules
    )
find_package(QGLViewer REQUIRED)

catkin_package(
    INCLUDE_DIRS include ${QGLVIEWER_INCLUDE_DIRS}
    LIBRARIES ${QGLVIEWER_LIBRARIES}
    CATKIN_DEPENDS kr_cmake_modules
    )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${QGLVIEWER_INCLUDE_DIRS}
    )

