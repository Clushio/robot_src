cmake_minimum_required(VERSION 3.8)
project(x2bot_teleop)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(cmd_vel_arbiter REQUIRED)
find_package(collision_monitor REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/NavConfig.srv"
  "srv/SetInt.srv"
  "srv/SetTagY.srv"
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(x2bot_joy_PXN src/x2bot_joy_PXN.cpp)
ament_target_dependencies(x2bot_joy_PXN geometry_msgs rclcpp sensor_msgs)

add_executable(saveLocation src/save_Location.cpp)
ament_target_dependencies(
  saveLocation rclcpp sensor_msgs tf2 tf2_geometry_msgs tf2_ros)

add_executable(runnav src/run_nav.cpp)
target_include_directories(runnav PRIVATE include)
ament_target_dependencies(
  runnav
  ament_index_cpp collision_monitor cmd_vel_arbiter diagnostic_msgs geometry_msgs nav2_msgs nav_msgs rclcpp
  rclcpp_action sensor_msgs std_msgs std_srvs tf2 tf2_geometry_msgs tf2_ros
  visualization_msgs)
target_link_libraries(runnav "${cpp_typesupport_target}")
target_link_libraries(runnav collision_monitor::collision_checker)

add_executable(TagCtl src/TagCtl_srv.cpp)
ament_target_dependencies(
  TagCtl cmd_vel_arbiter diagnostic_msgs geometry_msgs rclcpp tf2 tf2_geometry_msgs)
target_link_libraries(TagCtl "${cpp_typesupport_target}")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(terminal_goal_policy_test test/terminal_goal_policy_test.cpp)
  if(TARGET terminal_goal_policy_test)
    target_include_directories(terminal_goal_policy_test PRIVATE include)
  endif()
endif()

install(TARGETS x2bot_joy_PXN saveLocation runnav TagCtl
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
