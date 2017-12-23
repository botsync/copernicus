cmake_minimum_required(VERSION 2.8.3)
project(copernicus_teleoperator)

find_package(catkin REQUIRED COMPONENTS
  teleop_twist_joy
  teleop_twist_keyboard
)

catkin_package()