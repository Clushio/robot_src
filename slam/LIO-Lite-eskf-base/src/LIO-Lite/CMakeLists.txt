cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0074 NEW)
project(lio_lite)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Glog REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(TBB REQUIRED)
find_package(gflags REQUIRED)
find_package(yaml-cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Pose6D.msg"
  DEPENDENCIES geometry_msgs
)

add_definitions(-DROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")

option(WITH_IVOX_NODE_TYPE_PHC "Use PHC instead of default ivox node" OFF)
if(WITH_IVOX_NODE_TYPE_PHC)
  add_definitions(-DIVOX_NODE_TYPE_PHC)
endif()

add_library(${PROJECT_NAME}_core
  src/laser_mapping.cc
  src/static_map_filter.cc
  src/pointcloud_preprocess.cc
  src/options.cc
  src/utils.cc
)
target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${GLOG_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}_core
  ament_index_cpp
  builtin_interfaces
  geometry_msgs
  livox_ros_driver2
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(${PROJECT_NAME}_core
  ${cpp_typesupport_target}
  ${GLOG_LIBRARIES}
  ${PCL_LIBRARIES}
  TBB::tbb
  yaml-cpp
)

foreach(executable run_mapping_online run_location_online)
  add_executable(${executable} src/${executable}.cc)
  target_link_libraries(${executable} ${PROJECT_NAME}_core gflags)
  ament_target_dependencies(${executable} rclcpp)
endforeach()

foreach(executable test_split test_load)
  add_executable(${executable} src/test/${executable}.cc)
  target_link_libraries(${executable} ${PROJECT_NAME}_core gflags)
endforeach()

add_executable(test_static_map_filter src/test/test_static_map_filter.cc)
target_link_libraries(test_static_map_filter ${PROJECT_NAME}_core)

if(BUILD_TESTING)
  enable_testing()
  add_test(NAME static_map_filter_synthetic COMMAND test_static_map_filter)
endif()

install(TARGETS
  ${PROJECT_NAME}_core
  run_mapping_online
  run_location_online
  test_split
  test_load
  test_static_map_filter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
