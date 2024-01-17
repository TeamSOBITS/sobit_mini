cmake_minimum_required(VERSION 3.10.2)
project(sobit_mini_bringup)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
)



catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES sobit_pro_bringup
  CATKIN_DEPENDS rospy sensor_msgs
  #  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
