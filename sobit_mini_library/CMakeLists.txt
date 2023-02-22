cmake_minimum_required(VERSION 3.0.2)
project(sobit_mini_library)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  dynamixel_sdk
  hardware_interface
  roscpp
  rospy
  std_msgs
  sobit_common_msg
  trajectory_msgs
  geometry_msgs
  nav_msgs
  tf
  pybind11_catkin
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  controller_manager
  dynamixel_sdk
  hardware_interface
  roscpp
  rospy
  std_msgs
  sobit_common_msg
  trajectory_msgs
  geometry_msgs
  nav_msgs
  tf
  pybind11_catkin
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/sobit_turtlebot_controller.cpp
                            src/sobit_mini_controller.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

pybind_add_module(sobit_mini_module MODULE src/sobit_turtlebot_controller.cpp
                                           src/sobit_mini_controller.cpp
                                           src/sobit_mini_pybind.cpp)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
