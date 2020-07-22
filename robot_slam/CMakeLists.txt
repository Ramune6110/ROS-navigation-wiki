cmake_minimum_required(VERSION 3.0.2)
project(robot_slam)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  move_base
  roscpp 
  rospy 
  std_msgs 
  genmsg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_slam
#  CATKIN_DEPENDS roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

install(DIRECTORY launch maps config rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#add_executable(${PROJECT_NAME}_node src/robot_slam_node.cpp)
#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
