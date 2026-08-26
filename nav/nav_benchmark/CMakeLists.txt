cmake_minimum_required(VERSION 2.8.3)
project(nav_benchmark)

find_package(catkin REQUIRED COMPONENTS
  cmd_vel_arbiter
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
  tf2_ros
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    cmd_vel_arbiter
    diagnostic_msgs
    geometry_msgs
    nav_msgs
    rospy
    std_msgs
    tf2_ros
)

catkin_install_python(PROGRAMS
  scripts/navigation_benchmark.py
  scripts/analyze_results.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_metrics.py)
endif()
