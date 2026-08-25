cmake_minimum_required(VERSION 3.8)
project(livx2pcl)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(livox_repub livox2pcl.cpp)
target_include_directories(livox_repub PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(livox_repub ${PCL_LIBRARIES})
ament_target_dependencies(livox_repub
  livox_ros_driver2 pcl_conversions rclcpp sensor_msgs)

add_executable(livox_repub_ang livox2pcl_ang.cpp)
target_include_directories(livox_repub_ang PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(livox_repub_ang ${PCL_LIBRARIES})
ament_target_dependencies(livox_repub_ang
  livox_ros_driver2 pcl_conversions rclcpp sensor_msgs)

add_executable(livox_pcl_to_scan_clear livox_pcl_to_scan_clear.cpp)
ament_target_dependencies(livox_pcl_to_scan_clear rclcpp sensor_msgs)

install(TARGETS
  livox_repub
  livox_repub_ang
  livox_pcl_to_scan_clear
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_package()
