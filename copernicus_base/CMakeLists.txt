cmake_minimum_required(VERSION 2.8.3)
project(copernicus_base)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  rosserial_python
  sensor_msgs
  std_msgs
  copernicus_msgs
  tf
  um7
  imu_transformer
  imu_filter_madgwick
  laser_filters
  controller_manager
  hardware_interface
  diagnostic_updater
  diff_drive_controller
  robot_pose_publisher)

catkin_package(
INCLUDE_DIRS include
CATKIN_DEPENDS hardware_interface copernicus_msgs roscpp sensor_msgs
DEPENDS Boost modbus)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;")

add_library(copernicus_hardware src/copernicus_hardware.cpp)
add_executable(copernicus_base_node src/copernicus_base_node.cpp)
add_executable(copernicus_base src/copernicus_base.cpp)
target_link_libraries(copernicus_base_node ${catkin_LIBRARIES})
target_link_libraries(copernicus_base copernicus_hardware ${catkin_LIBRARIES} ${Boost_LIBRARIES})
