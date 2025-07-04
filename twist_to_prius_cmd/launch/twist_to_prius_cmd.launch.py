from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='twist_to_prius_cmd',
            executable='twist_to_prius_cmd.py',
            name='twist_to_prius_cmd',
            output='screen'
        ),
    ])

