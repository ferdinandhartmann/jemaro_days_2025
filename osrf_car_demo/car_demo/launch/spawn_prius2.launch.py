import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Get the urdf file
    urdf_file_name = "prius2.urdf"
    urdf_path = os.path.join(
        get_package_share_directory("prius_description"),
        "urdf", urdf_file_name
    )

    poses = {
        "0": {"x": 0.0, "y": 0.0, "z": 0.5, "Z": 0.0},
        "1": {"x": -78.4, "y": -407.75, "z": 0.5, "Z": 0.5},
        "2": {"x": 1.5, "y": -45.0, "z": 0.058, "Z": 1.57079632679},
    }
    
    # Launch configuration
    pose = LaunchConfiguration("pose", default="2")
    

    # Declare launch arguments
    declare_pose_cmd = DeclareLaunchArgument(
        "pose", default_value="2", description="Spawn Pose"
    )
    
    # robot_state_publisher node
    robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": open(urdf_path).read()}],
    )

    # spawn_entity node using -topic
    spawn_entity_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-x", TextSubstitution(text=str(poses["2"]["x"])),
            "-y", TextSubstitution(text=str(poses["2"]["y"])),
            "-z", TextSubstitution(text=str(poses["2"]["z"])),
            "-Y", TextSubstitution(text=str(poses["2"]["Z"])),
            "-entity", "prius"
        ],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(declare_pose_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_entity_cmd)

    return ld
