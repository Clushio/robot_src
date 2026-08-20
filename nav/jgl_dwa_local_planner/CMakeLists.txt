cmake_minimum_required(VERSION 2.8.3)
project(jgl_dwa_local_planner)

find_package(catkin REQUIRED
        COMPONENTS
            angles
            base_local_planner
            cmake_modules
            costmap_2d
            dynamic_reconfigure
            nav_core
            nav_msgs
            pluginlib
            sensor_msgs
            std_msgs
            roscpp
            tf2
            tf2_geometry_msgs
            tf2_ros
            tf
            visualization_msgs
        )

find_package(Eigen3 REQUIRED)
remove_definitions(-DDISABLE_LIBUSB-1.0)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    )
add_definitions(${EIGEN3_DEFINITIONS})

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/DWAPlanner.cfg
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES jgl_dwa_local_planner
    CATKIN_DEPENDS
        base_local_planner
        dynamic_reconfigure
        nav_msgs
        pluginlib
        sensor_msgs
        std_msgs
        roscpp
        tf2
        tf2_ros
        tf
        visualization_msgs
)

add_library(jgl_dwa_local_planner 
	src/dwa_planner.cpp 
	src/dwa_planner_ros.cpp
	src/trajectory_generator.cpp
	src/reference_path_manager.cpp
	src/path_follower.cpp
	src/speedPlan.cpp
	src/pursuit.cpp
	src/decay_obstacle_layer.cpp
	src/blind_clear_layer.cpp)
add_dependencies(jgl_dwa_local_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(jgl_dwa_local_planner ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(trajectory_generator_test test/trajectory_generator_test.cpp)
  if(TARGET trajectory_generator_test)
    set_target_properties(trajectory_generator_test PROPERTIES
      COMPILE_FLAGS "-DJGL_DWA_LOCAL_PLANNER_ENABLE_TEST_ACCESS")
    target_link_libraries(trajectory_generator_test
      jgl_dwa_local_planner
      ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(terminal_yaw_controller_test
    test/terminal_yaw_controller_test.cpp)
  catkin_add_gtest(reference_obstacle_policy_test
    test/reference_obstacle_policy_test.cpp)
endif()

install(TARGETS jgl_dwa_local_planner
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
       )

install(FILES blp_plugin.xml costmap_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
