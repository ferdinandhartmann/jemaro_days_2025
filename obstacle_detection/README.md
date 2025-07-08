# Obstacle Detection Package

Deteckt obstacles from LiDAR pointcloud based on RANSAC, Intensity and clustering and put them on a map.

## Usage

### Visualization
```bash 
rviz2 -d src/jemaro_days_2025/obstacle_detection/config/obstacle_detection.rviz
```

### Launch Real-Time Detection
To start the obstacle detection pipeline:
```bash
ros2 launch obstacle_detection launch_obstacle_detection_real.launch.py
```

### Replay Recorded Data
To replay a recorded ROS 2 bag file:
```bash
ros2 bag play -l --start-offset 8 --rate 0.6 src/data/rosbag2_2025_06_26-10_27_18/
```

## Notes
- Adjust file paths as needed for your system setup.