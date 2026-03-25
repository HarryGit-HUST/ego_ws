cmake_minimum_required(VERSION 3.0.2)
project(mission)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  tf
  serial
  actionlib
  move_base_msgs
  tf2_eigen
  tf2
  tf2_ros
  tf2_sensor_msgs
  image_transport
  pcl_ros
  pcl_conversions
  template
  quadrotor_msgs
)

catkin_package(
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(test_ego src/test_ego.cpp)
target_link_libraries(test_ego ${catkin_LIBRARIES})

add_executable(laser_to_worldframe src/laser_to_worldframe.cpp)
target_link_libraries(laser_to_worldframe ${catkin_LIBRARIES})