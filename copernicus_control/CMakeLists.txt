cmake_minimum_required(VERSION 2.8.3)
project(copernicus_control)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  diff_drive_controller
  joint_state_controller
  roscpp
  std_msgs
  copernicus_msgs
  twist_mux
  gazebo_ros_control
)

catkin_package()

include_directories(
		${catkin_INCLUDE_DIRS}
)

install(
        DIRECTORY launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)