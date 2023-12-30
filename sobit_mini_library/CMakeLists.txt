cmake_minimum_required(VERSION 3.10.2)
project(sobit_mini_library)

# set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")

## Compile as C++14, supported in ROS Noetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  #この塊はsobit_commonの命名が書き換えられたら消す
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
  tf2
  tf2_ros
  tf2_geometry_msgs
  pybind11_catkin

  #消したらコメントアウト消す
  # dynamixel_sdk
  # roscpp
  # rospy
  # std_msgs
  # tf2
  # tf2_ros
  # tf2_geometry_msgs
  # trajectory_msgs
  # geometry_msgs
  # nav_msgs
  # pybind11_catkin
  # sobits_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  # この塊はsobit_commonの命名が書き換えられたら消す
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
  tf2
  tf2_ros
  tf2_geometry_msgs
  pybind11_catkin

  #消したらコメントアウト消す
  # INCLUDE_DIRS include
  # LIBRARIES ${PROJECT_NAME}
  # CATKIN_DEPENDS
  # dynamixel_sdk
  # roscpp
  # rospy
  # std_msgs
  # tf2
  # tf2_ros
  # tf2_geometry_msgs
  # trajectory_msgs
  # geometry_msgs
  # nav_msgs
  # pybind11_catkin
  # sobits_msgs
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/sobit_mini_wheel_controller.cpp
                            src/sobit_mini_joint_controller.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}  ${catkin_EXPORTED_TARGETS})

pybind_add_module(sobit_mini_module MODULE src/sobit_mini_wheel_controller.cpp
                                           src/sobit_mini_joint_controller.cpp
                                           src/sobit_mini_pybind.cpp)
add_dependencies(sobit_mini_module ${catkin_EXPORTED_TARGETS})

#c++のテストプログラムを書いたらコメントアウト消す
# add_executable(test_control_arm example/test_control_arm.cpp
#                                 src/sobit_mini_joint_controller.cpp
#                                 src/sobit_mini_wheel_controller.cpp)
# target_link_libraries(test_control_arm ${catkin_LIBRARIES} ${PYTHON_LIBRARIES})
# add_dependencies(test_control_arm ${catkin_EXPORTED_TARGETS})

# add_executable(test_control_head example/test_control_head.cpp
#                                  src/sobit_mini_joint_controller.cpp
#                                  src/sobit_mini_wheel_controller.cpp)
# target_link_libraries(test_control_head ${catkin_LIBRARIES} ${PYTHON_LIBRARIES})
# add_dependencies(test_control_head ${catkin_EXPORTED_TARGETS})

# add_executable(test_control_wheel example/test_control_wheel.cpp
#                                  src/sobit_mini_joint_controller.cpp
#                                  src/sobit_mini_wheel_controller.cpp)
# target_link_libraries(test_control_wheel ${catkin_LIBRARIES} ${PYTHON_LIBRARIES})
# add_dependencies(test_control_wheel ${catkin_EXPORTED_TARGETS})

# add_executable(test_grasp example/test_grasp.cpp
#                           src/sobit_mini_joint_controller.cpp
#                           src/sobit_mini_wheel_controller.cpp)
# target_link_libraries(test_grasp ${catkin_LIBRARIES} ${PYTHON_LIBRARIES})
# add_dependencies(test_grasp ${catkin_EXPORTED_TARGETS})

# add_executable(test_place example/test_place.cpp
#                           src/sobit_mini_joint_controller.cpp
#                           src/sobit_mini_wheel_controller.cpp)
# target_link_libraries(test_place ${catkin_LIBRARIES} ${PYTHON_LIBRARIES})
# add_dependencies(test_place ${catkin_EXPORTED_TARGETS})

#これはいらない
####### add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


