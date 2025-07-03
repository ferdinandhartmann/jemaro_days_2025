cmake_minimum_required(VERSION 3.5)
project(odom_bag_to_path)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
  
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS script/odom_bag_to_path_py.py
  DESTINATION lib/${PROJECT_NAME})

ament_auto_package()
