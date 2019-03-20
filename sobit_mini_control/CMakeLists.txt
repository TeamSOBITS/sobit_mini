cmake_minimum_required(VERSION 2.8.3)
project(sobit_mini_control)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  dynamic_reconfigure
  dynamixel_sdk
  hardware_interface
  roscpp
  rospy
  sobit_common_msg
)
catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(sobit_mini_control src/sobit_common_dynamixel.cpp
                                  src/sobit_mini_dynamixel.cpp
                                  src/sobit_mini_control.cpp
                                  src/sobit_mini_hw.cpp)
target_link_libraries(sobit_mini_control
 ${catkin_LIBRARIES})