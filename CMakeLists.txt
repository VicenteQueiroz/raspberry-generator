cmake_minimum_required(VERSION 3.8)
project(raspberry_field)

find_package(ament_cmake REQUIRED)

install(DIRECTORY blender launch
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/generate_field.py
  scripts/ground_truth.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
