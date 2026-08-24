cmake_minimum_required(VERSION 3.8)
project(ranger_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(PROGRAMS
  scripts/bringup_can2usb.bash
  scripts/setup_can2usb.bash
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
