cmake_minimum_required(VERSION 2.8.3)
project(livx2pcl)

SET(CMAKE_BUILD_TYPE "Debug")


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -std=c++0x -std=c++14 -fexceptions -Wno-unused-local-typedefs")

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  livox_ros_driver2
  )

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
	${catkin_INCLUDE_DIRS} 
        ${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL OpenCV
  
)

add_executable(livox_repub livox2pcl.cpp)
target_link_libraries(livox_repub ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})


add_executable(livox_repub_ang livox2pcl_ang.cpp)
target_link_libraries(livox_repub_ang ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(livox_pcl_to_scan_clear livox_pcl_to_scan_clear.cpp)
target_link_libraries(livox_pcl_to_scan_clear ${catkin_LIBRARIES})


