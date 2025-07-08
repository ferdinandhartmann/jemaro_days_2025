from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    path_planning_node = Node(
        package='path_planning',
        executable='path_planning',
        name='path_planning_node',
        output='screen'
    )

    map_publisher_node = Node(
        package='path_planning',
        executable='map_publisher',
        name='map_publisher_node',
        output='screen'
    )

    return LaunchDescription([
        path_planning_node,
        # map_publisher_node
    ])
