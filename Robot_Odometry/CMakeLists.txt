cmake_minimum_required(VERSION 3.0.2)
project(Robot_Odometry)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  tf
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(Odometry_publisher src/Odometry_publisher.cpp)
add_dependencies(Odometry_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Odometry_publisher ${catkin_LIBRARIES})