cmake_minimum_required(VERSION 3.0.2)
project(cmd_vel_arbiter)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(cmd_vel_arbiter_node src/cmd_vel_arbiter_node.cpp)
target_link_libraries(cmd_vel_arbiter_node ${catkin_LIBRARIES})
add_dependencies(cmd_vel_arbiter_node ${catkin_EXPORTED_TARGETS})

install(TARGETS cmd_vel_arbiter_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
