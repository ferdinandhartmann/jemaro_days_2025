# jemaro_days_2025


# Gazebo
The Gazebo prius environment is based on https://github.com/mattborghi/osrf_car_demo

Launch the Gazebo simulator with the straight road world and the prius:

'ros2 launch car_demo demo2.launch.py'

Convert twist messages to the prius control messages:

ros2 launch twist_to_prius_cmd.launch.py

Launch a basic pure pursuit path following algorithm:

ros2 launch car_control car_control.launch.py

Publish the path of the center of the lane and of the right and left sides of the road:

ros2 bag play path_prius

ros2 bag play path_right_prius --remap /path:=path_right

ros2 bag play path_left_prius --remap /path:=path_left

# LiDAR dataset
The pointcloud downsampling is based on https://github.com/LihanChen2004/pointcloud_downsampling

https://uncloud.univ-nantes.fr/index.php/s/tkEwQcM7qGMj9wp

rviz2 -d jemaro.rviz

ros2 launch pointcloud_downsampling pointcloud_downsampling.launch.py

ros2 bag play path_zoe --remap /ZOE3/path_follower/setPath:=path

ros2 bag play path_left_zoe --remap /ZOE3/path_follower/setPath:=path_left

ros2 bag play path_right_zoe --remap /ZOE3/path_follower/setPath:=path_right

ros2 bag play rosbag2_2025_06_26-10_27_18





