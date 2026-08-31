cmake_minimum_required(VERSION 3.8)
project(nav_benchmark)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(
  ${PROJECT_NAME}
  PACKAGE_DIR src/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/navigation_benchmark.py
  scripts/analyze_results.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_metrics test/test_metrics.py TIMEOUT 30)
endif()

ament_package()
