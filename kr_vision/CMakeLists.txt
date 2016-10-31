cmake_minimum_required(VERSION 2.8.3)
project(kr_vision)

find_package(catkin REQUIRED COMPONENTS
  kr_math
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS kr_math
)

include_directories(include ${catkin_INCLUDE_DIRS})

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  # todo: test stuff
endif()
