cmake_minimum_required(VERSION 3.0.2)
project(Robot_point_cloud_publisher)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(point_cloud_publisher src/point_cloud_publisher.cpp)
add_dependencies(point_cloud_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(point_cloud_publisher ${catkin_LIBRARIES})
