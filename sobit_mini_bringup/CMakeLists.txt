cmake_minimum_required(VERSION 3.10.2)
project(sobit_mini_bringup)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  # roscpp
  # rospy
  # geometry_msgs
  # std_msgs
  # tf2
  # dynamixel_sdk
  # sobit_common_msg
  rospy
  sensor_msgs
)



catkin_package(
  # CATKIN_DEPENDS
  # roscpp
  # rospy
  # std_msgs
  # trajectory_msgs

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
