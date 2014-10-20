cmake_minimum_required(VERSION 2.8.3)
project(kr_math)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs)

# Find Eigen
find_package(cmake_modules)
find_package(Eigen REQUIRED)

include("cmake/extras.cmake")
include_directories(include ${EIGEN_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs
  CFG_EXTRAS "extras.cmake"
  DEPENDS Eigen
)

# Mark header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# gtest
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(gtest_all test/gtest_all.cpp
    test/SO3_test.cpp
    test/pose_test.cpp
    )
  target_link_libraries(gtest_all
    ${catkin_LIBRARIES}
    ${GTEST_LIBRARIES})
endif()
