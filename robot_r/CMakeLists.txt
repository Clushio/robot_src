cmake_minimum_required(VERSION 3.8)
project(robot_r)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_library(robot_r_anav_panel SHARED
  src/anav_panel.cpp
  src/record_pose_tool.cpp
  include/robot_r/anav_panel.h
  include/robot_r/record_pose_tool.h
)
target_include_directories(robot_r_anav_panel
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(robot_r_anav_panel
  pluginlib
  rviz_common
  rviz_default_plugins
)
target_link_libraries(robot_r_anav_panel Qt5::Widgets)

pluginlib_export_plugin_description_file(
  rviz_common plugins/anav_panel_plugin.xml)

install(TARGETS robot_r_anav_panel
  EXPORT export_robot_r_anav_panel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugins/anav_panel_plugin.xml
  DESTINATION share/${PROJECT_NAME}/plugins
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "display_point_cloud_ROS1.rviz" EXCLUDE
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY lio
  DESTINATION share/${PROJECT_NAME}
  PATTERN "mid360.yaml" EXCLUDE
  PATTERN "hesaixt16.yaml" EXCLUDE
)

install(PROGRAMS
  scripts/mm3v_serial_reader.py
  scripts/tcpserver.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_robot_r_anav_panel HAS_LIBRARY_TARGET)
ament_export_dependencies(pluginlib rviz_common rviz_default_plugins)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_mm3v_parser test/test_mm3v_parser.py
    TIMEOUT 30
  )
  ament_add_pytest_test(
    test_tcpserver test/test_tcpserver.py
    TIMEOUT 30
  )
endif()

ament_package()
