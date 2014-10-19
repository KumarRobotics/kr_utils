# Enable c++11 flag
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_definitions("-DKR_MATH_BUILD_ROS")
add_definitions("-DKR_MATH_ROS_CONVERSIONS -DKR_MATH_YAMLCPP_CONVERSIONS")
add_definitions("-DKR_MATH_ROS_$ENV{ROS_DISTRO}")
