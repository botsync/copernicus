cmake_minimum_required(VERSION 2.8.3)
project(copernicus_teleoperator)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  teleop_twist_joy
  teleop_twist_keyboard
  sensor_msgs
  geometry_msgs
  copernicus_msgs
  actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    joy_teleop
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(joy_teleop src/joy_teleop.cpp)

add_dependencies(joy_teleop copernicus_teleoperator_gencpp)

add_executable(joy_teleop_node src/joy_teleop_node.cpp)

target_link_libraries(joy_teleop_node joy_teleop ${catkin_LIBRARIES})
