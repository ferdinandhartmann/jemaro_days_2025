from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_path = get_package_share_directory('obstacle_detection')
    rviz_config_path = os.path.join(pkg_path, 'config', 'obstacle_detection.rviz')
    params_file = os.path.join(pkg_path, 'config', 'obstacle_params.yaml')

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )


    downsampling_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('pointcloud_downsampling'),
            'launch',
            'pointcloud_downsampling.launch.py'
        ))
    )

    obstacle_node = Node(
        package='obstacle_detection',
        executable='obstacle_detection_node',
        name='obstacle_detector',
        output='screen',
        parameters=[params_file]
    )

    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_custom_map',
        arguments=['-980', '-760', '0', '0', '0', '0', 'custom_map', 'map'],
        output='screen'
    )

    path_planning_node = Node(
        package='path_planning',
        executable='path_planning',
        name='path_planning_node',
        output='screen',
    )

    return LaunchDescription([
        # rviz2,
        downsampling_launch,
        obstacle_node,
        # static_tf_publisher,
        path_planning_node
    ])
