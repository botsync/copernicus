cmake_minimum_required(VERSION 2.8.3)
project(copernicus_teleoperator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  copernicus_msgs
  teleop_twist_joy
  teleop_twist_keyboard
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    copernicus_joy_teleop
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(copernicus_joy_teleop src/joy_teleop.cpp)
add_executable(copernicus_joy_teleop_node src/joy_teleop_node.cpp)
target_link_libraries(copernicus_joy_teleop_node copernicus_joy_teleop ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(TARGETS
        copernicus_joy_teleop
        copernicus_joy_teleop_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
