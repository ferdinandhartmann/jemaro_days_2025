cmake_minimum_required(VERSION 3.5)
project(twist_to_prius_cmd)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
  
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS script/twist_to_prius_cmd.py
  DESTINATION lib/${PROJECT_NAME})

ament_auto_package()
