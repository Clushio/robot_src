cmake_minimum_required(VERSION 3.8)
project(collision_monitor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cmd_vel_arbiter REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

set(PACKAGE_DEPENDENCIES
  cmd_vel_arbiter
  diagnostic_msgs
  geometry_msgs
  nav2_costmap_2d
  nav_msgs
  rclcpp
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

add_library(collision_checker SHARED src/collision_checker.cpp)
target_include_directories(collision_checker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(collision_checker
  geometry_msgs nav2_costmap_2d nav_msgs tf2 tf2_geometry_msgs)

add_executable(collision_monitor_node src/collision_monitor_node.cpp)
target_include_directories(collision_monitor_node PRIVATE include)
target_link_libraries(collision_monitor_node collision_checker)
ament_target_dependencies(collision_monitor_node ${PACKAGE_DEPENDENCIES})

install(TARGETS collision_checker
  EXPORT collision_checkerTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(TARGETS collision_monitor_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_gtest(collision_checker_test test/collision_checker_test.cpp)
  target_link_libraries(collision_checker_test collision_checker)
  ament_target_dependencies(collision_checker_test geometry_msgs nav_msgs)
  add_launch_test(test/collision_monitor_node_test.py TIMEOUT 45)
endif()

ament_export_targets(collision_checkerTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  geometry_msgs nav2_costmap_2d nav_msgs tf2 tf2_geometry_msgs)
ament_export_include_directories(include)
ament_package()
