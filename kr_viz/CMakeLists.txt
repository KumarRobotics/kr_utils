cmake_minimum_required(VERSION 2.8.3)
project(kr_viz)

add_definitions(-std=c++11 -Wall)

find_package(catkin REQUIRED COMPONENTS
    kr_cmake_modules
    )
find_package(OpenCV REQUIRED)
find_package(QGLViewer REQUIRED)

catkin_package(
    INCLUDE_DIRS include ${QGLVIEWER_INCLUDE_DIRS}
    LIBRARIES ${QGLVIEWER_LIBRARIES}
    CATKIN_DEPENDS kr_cmake_modules
    DEPENDS OpenCV
    )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${QGLVIEWER_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    )

add_executable(viewer
    src/qglviewer/viewer_main.cpp
    src/qglviewer/camera_viewer.cpp
    src/qglviewer/camera.cpp
    src/qglviewer/texture.cpp
    )
target_link_libraries(viewer
    ${QGLVIEWER_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )
