cmake_minimum_required(VERSION 3.0.2)
project(sobit_mini_description)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  dynamic_reconfigure
  pcl_conversions
  pcl_ros
  tf
  message_generation
  sensor_msgs
)

##add_message_files(
## FILES
## dynamixel_pos_id.msg
## dynamixel_state_array.msg
## dynamixel_state.msg
## location_stock.msg
##)

##add_service_files(
## FILES
## odom_base.srv
## grasp_ctrl.srv
## waypoint_nav.srv
##)

generate_messages(
 DEPENDENCIES
 std_msgs
 geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

##add_executable(pose_saver_edu src/pose_saver_tf.cpp)
##target_link_libraries(pose_saver_edu ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

##add_executable(waypoint_nav_edu src/waypoint_nav.cpp)
##target_link_libraries(waypoint_nav_edu ${catkin_LIBRARIES})

##add_executable(pcl_scan_creater_edu src/pcl_scan_creater.cpp)
##target_link_libraries(pcl_scan_creater_edu ${catkin_LIBRARIES})
