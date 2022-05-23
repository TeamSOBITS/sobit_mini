cmake_minimum_required(VERSION 3.0.2)
project(sobit_mini_bringup)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  std_msgs
  tf
  dynamixel_sdk
  sobit_common_msg
)



catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  trajectory_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
