cmake_minimum_required(VERSION 3.8)
project(anav_ranger)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  aNAV_ranger.py
  Tag_ros.py
  build_topology.py
  mainlo.py
  mm3v_serial_reader.py
  pathRecord.py
  stpbuild.py
  stpnav_B.py
  stpnav_C.py
  stpnav_D.py
  tagReader.py
  tcpserver.py
  transfer_robot_positions.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  fault_center.py
  legacy_nav_gui.py
  loop_start_policy.py
  ros2_runtime.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_build_topology test_build_topology.py TIMEOUT 30)
  ament_add_pytest_test(test_loop_start_policy test_loop_start_policy.py TIMEOUT 30)
  ament_add_pytest_test(test_ros2_runtime test_ros2_runtime.py TIMEOUT 30)
  ament_add_pytest_test(
    test_migration_guards test_migration_guards.py TIMEOUT 30)
  ament_add_pytest_test(test_path_record test_path_record.py TIMEOUT 30)
endif()

ament_package()
