cmake_minimum_required(VERSION 3.0.2)
project(Robot_setup_tf)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES Robot_setup_tf
  CATKIN_DEPENDS geometry_msgs roscpp tf
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})

add_executable(tf_listener src/tf_listener.cpp)
target_link_libraries(tf_listener ${catkin_LIBRARIES})
