cmake_minimum_required(VERSION 3.8)
project(obstacle_detection)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io kdtree registration visualization)

add_library(ground_detector SHARED src/GroundDetector.cpp)
target_include_directories(ground_detector
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(ground_detector ${PCL_LIBRARIES})
ament_target_dependencies(ground_detector pcl_conversions)

add_executable(obstacle_detection src/main_ls.cpp)
target_include_directories(obstacle_detection PRIVATE include ${PCL_INCLUDE_DIRS})
target_link_libraries(obstacle_detection ground_detector ${PCL_LIBRARIES})
ament_target_dependencies(obstacle_detection rclcpp sensor_msgs std_msgs pcl_conversions)

add_executable(obstacle_detection_velodyne src/main_velodyne.cpp)
target_include_directories(obstacle_detection_velodyne PRIVATE include ${PCL_INCLUDE_DIRS})
target_link_libraries(obstacle_detection_velodyne ground_detector ${PCL_LIBRARIES})
ament_target_dependencies(obstacle_detection_velodyne rclcpp sensor_msgs std_msgs pcl_conversions)

install(TARGETS ground_detector obstacle_detection obstacle_detection_velodyne
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(ground_detector)
ament_export_dependencies(pcl_conversions PCL)

ament_package()
