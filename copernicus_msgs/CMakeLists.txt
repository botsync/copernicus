cmake_minimum_required(VERSION 2.8.3)
project(copernicus_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  Velocities.msg
  PID.msg
  PBStatusFlags.msg
  SubsPBFlags.msg
  BMS.msg
  Sonar.msg
  )

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
)
