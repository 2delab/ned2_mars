
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from pathlib import Path


def generate_launch_description():
    """
    Launch the complete N-robot MoveIt2 demo

    This demo uses:
    - Fake ROS2 control system (no hardware needed)
    - Joint state prefixer for N-robot coordination
    - MoveIt2 for motion planning
    - RViz for visualization
    """

    # Include main nrobot_moveit_launch.py
    nrobot_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("niryo_ned2_nrobot_moveit_config"),
                "/launch/nrobot_moveit_launch.py",
            ]
        )
    )

    return LaunchDescription(
        [
            nrobot_moveit_launch,
        ]
    )
