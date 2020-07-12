cmake_minimum_required(VERSION 3.0.2)
project(Robot_navigation_goal)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  roscpp
  tf
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(navigation_goal src/navigation_goal.cpp)
add_dependencies(navigation_goal ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(navigation_goal ${catkin_LIBRARIES})
