cmake_minimum_required(VERSION 3.0.2)
project(Robot_2D_navigation)

find_package(catkin REQUIRED COMPONENTS
  move_base
)

catkin_package()
install(DIRECTORY launch maps config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#include_directories(include ${catkin_INCLUDE_DIRS})

