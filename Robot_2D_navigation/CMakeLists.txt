cmake_minimum_required(VERSION 3.0.2)
project(Robot_2D_navigation)

find_package(catkin REQUIRED COMPONENTS
  move_base
  roscpp 
  rospy 
  std_msgs 
  genmsg
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

install(DIRECTORY launch maps config rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(robot_interface src/robot_interface.cpp src/torque_calculater.cpp)
target_link_libraries(robot_interface ${catkin_LIBRARIES})

#add_executable(robot_interface src/robot_interface.cpp)
#add_dependencies(robot_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(robot_interface ${catkin_LIBRARIES})

#add_executable(torque_calculater src/torque_calculater.cpp)
#add_dependencies(torque_calculater ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(torque_calculater ${catkin_LIBRARIES})


