cmake_minimum_required(VERSION 3.0.2)
project(Robot_laser_scan)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(laser_scan_publisher src/laser_scan_publisher.cpp)
add_dependencies(laser_scan_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(laser_scan_publisher ${catkin_LIBRARIES})
