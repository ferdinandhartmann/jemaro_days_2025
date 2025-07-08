cmake_minimum_required(VERSION 3.8)
project(obstacle_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation search)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(obstacle_detection_node src/obstacle_detection.cpp)
ament_target_dependencies(obstacle_detection_node
  rclcpp
  std_msgs
  sensor_msgs
  visualization_msgs
  pcl_conversions
  pcl_ros
  PCL
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)


# add_executable(create_map src/create_map.cpp)
# target_include_directories(create_map PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(create_map PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# ament_target_dependencies(create_map
#   rclcpp
#   geometry_msgs
#   sensor_msgs
#   nav_msgs
#   tf2
#   tf2_ros
#   tf2_geometry_msgs
# )


install(TARGETS
    obstacle_detection_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
    launch 
    config
    DESTINATION share/${PROJECT_NAME}
)


ament_package()
