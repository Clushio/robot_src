cmake_minimum_required(VERSION 3.8)
project(jgl_dwa_local_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(anav_interfaces REQUIRED)
find_package(angles REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(dwb_core REQUIRED)
find_package(dwb_plugins REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/dwa_planner_ros.cpp
  src/trajectory_generator.cpp
  src/reference_path_manager.cpp
  src/path_follower.cpp
  src/speedPlan.cpp
  src/pursuit.cpp
  src/decay_obstacle_layer.cpp
  src/blind_clear_layer.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME}
  angles
  dwb_core
  dwb_plugins
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav_msgs
  pluginlib
  rcl_interfaces
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs anav_interfaces)
target_link_libraries(${PROJECT_NAME} Boost::thread)

pluginlib_export_plugin_description_file(nav2_core blp_plugin.xml)
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugin.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME}
  PATTERN "dwa_planner.h" EXCLUDE)
install(FILES blp_plugin.xml costmap_plugin.xml
  DESTINATION share/${PROJECT_NAME})
install(FILES
  param/dwa_local_planner_params.yaml
  param/controller_server_example.yaml
  DESTINATION share/${PROJECT_NAME}/param)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(trajectory_generator_test test/trajectory_generator_test.cpp)
  if(TARGET trajectory_generator_test)
    target_compile_definitions(trajectory_generator_test PRIVATE
      JGL_DWA_LOCAL_PLANNER_ENABLE_TEST_ACCESS)
    target_link_libraries(trajectory_generator_test ${PROJECT_NAME})
    ament_target_dependencies(trajectory_generator_test
      geometry_msgs nav_msgs rclcpp tf2 tf2_geometry_msgs)
  endif()

  ament_add_gtest(terminal_yaw_controller_test
    test/terminal_yaw_controller_test.cpp)
  if(TARGET terminal_yaw_controller_test)
    target_include_directories(terminal_yaw_controller_test PRIVATE include)
  endif()

  ament_add_gtest(reference_obstacle_policy_test
    test/reference_obstacle_policy_test.cpp)
  if(TARGET reference_obstacle_policy_test)
    target_include_directories(reference_obstacle_policy_test PRIVATE include)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  angles dwb_core dwb_plugins geometry_msgs nav2_core nav2_costmap_2d
  nav_msgs pluginlib rcl_interfaces rclcpp rclcpp_lifecycle sensor_msgs std_msgs
  tf2 tf2_geometry_msgs tf2_ros visualization_msgs anav_interfaces)
ament_package()
