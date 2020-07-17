cmake_minimum_required(VERSION 3.0.2)
project(Robot_2D_navigation)

find_package(catkin REQUIRED COMPONENTS
  move_base
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

#add_executable(${PROJECT_NAME}_node src/my_robot_name_2dnav_node.cpp)
#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

