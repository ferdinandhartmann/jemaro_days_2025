from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='path_planning',
            executable='path_planning',
            name='path_planning',
            output='screen'
        )
    ])
