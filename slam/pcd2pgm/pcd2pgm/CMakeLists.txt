cmake_minimum_required(VERSION 3.8)
project(pcd2pgm)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io)

add_executable(pcd2topic src/test.cpp)
target_include_directories(pcd2topic SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
ament_target_dependencies(pcd2topic
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
)
target_link_libraries(pcd2topic ${PCL_LIBRARIES})

install(TARGETS pcd2topic
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
