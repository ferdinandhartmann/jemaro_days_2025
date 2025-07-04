# jemaro_days_2025


# Gazebo

ros2 launch car_demo demo2.launch.py

ros2 launch twist_to_prius_cmd.launch.py
ros2 launch car_control car_control.launch.py
ros2 bag play path_prius
ros2 bag play path_right_prius --remap /path:=path_right
ros2 bag play path_left_prius --remap /path:=path_left

# LiDAR dataset
rviz2 -d jemaro.rviz
ros2 launch pointcloud_downsampling pointcloud_downsampling.launch.py
ros2 bag play rosbag2_2025_06_26-10_27_18





