cmake_minimum_required(VERSION 2.8.3)
project(copernicus_description)

find_package(catkin REQUIRED COMPONENTS
  roslaunch
  urdf
  xacro
  hector_gazebo_plugins
  velodyne_description
)

catkin_package()

roslaunch_add_file_check(launch)

install(
  DIRECTORY launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)