cmake_minimum_required(VERSION 2.8.3)
project(copernicus_control)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  diff_drive_controller
  joint_state_controller
  dynamic_reconfigure
  roscpp
  std_msgs
  copernicus_msgs
  twist_mux
)

generate_dynamic_reconfigure_options(
  config/copernicusPID.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS controller_manager diff_drive_controller joint_state_controller dynamic_reconfigure roscpp std_msgs copernicus_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(pid_core src/pid_core.cpp)
add_executable(pid_reconfigure src/pid_reconfigure.cpp)
target_link_libraries(pid_reconfigure pid_core ${catkin_LIBRARIES})
add_dependencies(pid_reconfigure ${PROJECT_NAME}_gencpp)
