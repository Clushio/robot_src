cmake_minimum_required(VERSION 3.8)
project(cmd_vel_arbiter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ranger_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArbitratedCommand.msg"
  "srv/FinishMotion.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

add_executable(cmd_vel_arbiter_node src/cmd_vel_arbiter_node.cpp)
ament_target_dependencies(cmd_vel_arbiter_node
  diagnostic_msgs
  geometry_msgs
  ranger_msgs
  rclcpp
  rclcpp_action
  std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(cmd_vel_arbiter_node "${cpp_typesupport_target}")

install(TARGETS cmd_vel_arbiter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
