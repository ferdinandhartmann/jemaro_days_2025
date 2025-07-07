from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # RViz config path
    rviz_config_path = os.path.join(
        get_package_share_directory('obstacle_detection'),
        'config',
        'real.rviz'
    )

    # Launch RViz
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # Include pointcloud_downsampling launch
    downsampling_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('pointcloud_downsampling'),
            'launch',
            'pointcloud_downsampling.launch.py'
        ))
    )

    return LaunchDescription([
        rviz2,
        downsampling_launch
    ])
