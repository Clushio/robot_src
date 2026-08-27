cmake_minimum_required(VERSION 3.8)
project(mxb_move_base)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(mxb_move_base src/move_base_compat.cpp)
ament_target_dependencies(mxb_move_base
  geometry_msgs
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
)

install(TARGETS mxb_move_base
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_copyright(src/move_base_compat.cpp launch/navigation.launch.py)
  ament_cpplint(src/move_base_compat.cpp)
  ament_flake8(launch/navigation.launch.py)
  ament_lint_cmake(CMakeLists.txt)
  ament_pep257(launch/navigation.launch.py)
  ament_uncrustify(src/move_base_compat.cpp)
  ament_xmllint(
    package.xml
    behavior_trees/navigate_through_poses.xml
    behavior_trees/navigate_to_pose_fixed_route.xml
    behavior_trees/navigate_to_pose_replanning.xml
  )
endif()

ament_package()
