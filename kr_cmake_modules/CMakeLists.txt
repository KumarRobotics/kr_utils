cmake_minimum_required(VERSION 2.8.3)
project(kr_cmake_modules)

find_package(catkin REQUIRED)

catkin_package(CFG_EXTRAS kr_cmake_modules-extras.cmake)

# TODO(Kartik): The following does not work, need to find the correct way
#install(FILES kr_cmake_modules-extras.cmake
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake
#  )

install(DIRECTORY cmake/Modules
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake
  )
