cmake_minimum_required(VERSION 2.8.12)
project(rviz_ba_viewer)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wunused -std=c++11")

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

# Qt
add_definitions(-DQT_NO_KEYWORDS)

## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 COMPONENTS QtCore QtGui)
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()


add_message_files(DIRECTORY msg)
generate_messages(DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  rviz
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  kr_cmake_modules
  image_geometry
  message_runtime)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIR}
  ${Ogre_INCLUDE_DIRS}
)

set(${PROJECT_NAME}_MOC_HEADERS
  include/rviz_ba_viewer/ba_display.h
)

if(rviz_QT_VERSION VERSION_LESS "5")
  qt4_wrap_cpp(${PROJECT_NAME}_MOCSrcs ${${PROJECT_NAME}_MOC_HEADERS})
else()
  qt5_wrap_cpp(${PROJECT_NAME}_MOCSrcs ${${PROJECT_NAME}_MOC_HEADERS})
endif()

add_library(${PROJECT_NAME}
  src/ba_display.cpp
  src/keyframe_object.cpp
  ${${PROJECT_NAME}_MOCSrcs}
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Ogre_LIBRARIES}
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
