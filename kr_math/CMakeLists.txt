cmake_minimum_required(VERSION 2.8.3)
project(kr_math)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules geometry_msgs)

# Find Eigen
find_package(Eigen REQUIRED)

list(APPEND INCLUDE_DIRS "include" ${EIGEN_INCLUDE_DIRS})
list(APPEND PACKAGE_DEPENDS "Eigen")

# GTSAM
find_package(GTSAM)
if(${GTSAM_FOUND})
  set(GTSAM_INCLUDE_DIRS ${GTSAM_INCLUDE_DIR})
  set(GTSAM_LIBRARIES gtsam)

  list(APPEND INCLUDE_DIRS ${GTSAM_INCLUDE_DIRS})
  list(APPEND EXTERNAL_LIBS ${GTSAM_LIBRARIES})
  list(APPEND PACKAGE_DEPENDS "GTSAM")
endif()

configure_file("cmake/extras.cmake.in" "cmake/extras.cmake" @ONLY)
include("${CMAKE_CURRENT_BINARY_DIR}/cmake/extras.cmake")

include_directories(${INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs
  CFG_EXTRAS "extras.cmake"
  DEPENDS ${PACKAGE_DEPENDS}
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
    ${GTEST_LIBRARIES}
    ${EXTERNAL_LIBS})
endif()
