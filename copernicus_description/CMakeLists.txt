cmake_minimum_required(VERSION 2.8.3)
project(copernicus_description)

find_package(catkin REQUIRED COMPONENTS
  roslaunch
  urdf
  xacro
  velodyne_description
)

catkin_package()

include_directories(
 ${catkin_INCLUDE_DIRS}
)

roslaunch_add_file_check(launch)

install(
  DIRECTORY launch meshes rviz_params urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
