cmake_minimum_required(VERSION 3.10.2)
project(sobit_mini_control)

add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  # この塊はsobit_commonの命名が書き換えられたら消す
  controller_manager
  dynamic_reconfigure
  dynamixel_sdk
  hardware_interface
  roscpp
  rospy
  sobit_common_msg

  # 消したらコメントアウト消す
  # dynamixel_sdk
  # sobits_msgs
  # sobits_common
)

# この塊はsobit_commonの命名が書き換えられたら消す
catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 消したらコメントアウト消す
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
# catkin_package(
#   # INCLUDE_DIRS include
#   LIBRARIES ${PROJECT_NAME}
#   CATKIN_DEPENDS
#   dynamixel_sdk
#   sobits_msgs
#   sobits_common
# )