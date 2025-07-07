from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    car_demo_dir = get_package_share_directory('car_demo')
    twist_cmd_dir = get_package_share_directory('twist_to_prius_cmd')
    car_control_dir = get_package_share_directory('car_control')

    return LaunchDescription([
        # Launch Gazebo straight road simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(car_demo_dir, 'launch', 'straight_road.launch.py')
            )
        ),

        # Launch Twist to Prius command converter
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(twist_cmd_dir, 'launch', 'twist_to_prius_cmd.launch.py')
            )
        ),

        # Launch Pure Pursuit controller
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(car_control_dir, 'launch', 'car_control.launch.py')
            )
        ),
    ])
