cmake_minimum_required(VERSION 2.8.3)
project(rviz_ba_viewer)

#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  kr_cmake_modules
  image_geometry
  message_generation
)

# OpenCV
find_package(OpenCV REQUIRED)
find_package(Ogre REQUIRED)

add_message_files(DIRECTORY msg)
generate_messages(DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS
  roscpp
  rviz
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  kr_cmake_modules
  image_geometry
  message_runtime)

set(${PROJECT_NAME}_SOURCES
  src/ba_display.cpp
  src/keyframe_object.cpp
)

set(${PROJECT_NAME}_HEADERS
  include/rviz_ba_viewer/ba_display.h
)

# invoke MOC and UI
QT4_WRAP_CPP(${PROJECT_NAME}_MOCSrcs ${${PROJECT_NAME}_HEADERS})

# QT Declarations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${OpenCV_INCLUDE_DIR}
  ${Qt4_INCLUDE_DIR}
  ${Ogre_INCLUDE_DIRS}
)

add_definitions("-Wall -Wunused -std=c++11")

set(PROJECT_SOURCE_FILES
  ${${PROJECT_NAME}_SOURCES}
  ${${PROJECT_NAME}_MOCSrcs}
)

add_library(${PROJECT_NAME}
  ${PROJECT_SOURCE_FILES}
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencpp
)

target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OGRE_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    )

install(FILES plugin_description.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )
