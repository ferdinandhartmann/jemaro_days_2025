from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='odom_bag_to_path',
            executable='odom_bag_to_path_py.py',
            name='odom_bag_to_path',
            output='screen',
            parameters=[{'dataset': 'path',
                'topic': '/prius/ground_truth',
                'frequency': 1,
                'downsample_n': 100}]
        ),
    ])

