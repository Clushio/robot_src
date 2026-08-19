cmake_minimum_required(VERSION 3.0.2)
project(collision_monitor)

add_compile_options(-std=c++11 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  cmd_vel_arbiter
  costmap_2d
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES collision_checker
  CATKIN_DEPENDS cmd_vel_arbiter costmap_2d diagnostic_msgs geometry_msgs
                 nav_msgs roscpp std_msgs tf2 tf2_geometry_msgs tf2_ros
                 visualization_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(collision_checker src/collision_checker.cpp)
target_link_libraries(collision_checker ${catkin_LIBRARIES})
add_dependencies(collision_checker ${catkin_EXPORTED_TARGETS})

add_executable(collision_monitor_node src/collision_monitor_node.cpp)
target_link_libraries(collision_monitor_node collision_checker ${catkin_LIBRARIES})
add_dependencies(collision_monitor_node ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(collision_checker_test test/collision_checker_test.cpp)
  if(TARGET collision_checker_test)
    target_link_libraries(collision_checker_test collision_checker ${catkin_LIBRARIES})
  endif()
  add_rostest(test/collision_monitor_node.test)
endif()

install(TARGETS collision_checker collision_monitor_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
