cmake_minimum_required(VERSION 2.8.3)
project(copernicus_base)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  ros_arduino_msgs
  roscpp
  rospy
  rosserial_python
  sensor_msgs
  std_msgs
  tf
  um7
  imu_transformer
  imu_filter_madgwick
  mavros)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
add_executable(copernicus_base_node src/copernicus_base_node.cpp)
target_link_libraries(copernicus_base_node ${catkin_LIBRARIES})
