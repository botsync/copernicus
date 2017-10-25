cmake_minimum_required(VERSION 2.8.3)
project(copernicus_navigation)

find_package(catkin REQUIRED COMPONENTS
  amcl
  move_base
  gmapping
)