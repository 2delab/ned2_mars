from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch file for vision system with ArUco detection and camera transforms."""

    # ArUco detection node for mono camera
    aruco_detection_node = Node(
        package="niryo_ned_vision",
        executable="arucodetection",
        name="aruco_detection",
        output="both",
    )

    # ArUco detection node for D435 camera
    d435_aruco_detection_node = Node(
        package="niryo_ned_vision",
        executable="d435_arucodetection",
        name="d435_aruco_detection",
        output="both",
    )

    # Static transform publisher for D435 camera
    d435_camera_tf_broadcaster = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0.010573",
            "0.011054",
            "-0.070225",
            "0.164512",
            "-3.14",
            "1.7453",
            "arm_1_wrist_link",
            "d435_camera"
        ],
        name="d435_camera_tf_broadcaster",
        output="both",
    )


    '''
        # Static transform publisher for D435 camera
    d435_camera_tf_broadcaster = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0.0385",
            "0",
            "0",
            "0",
            "0",
            "1.7453",
            "arm_1_wrist_link",
            "d435_camera",
        ],
        name="d435_camera_tf_broadcaster",
        output="both",
    )

    '''


    # Static transform publisher for mono camera
    mono_camera_tf_broadcaster = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0.0385",
            "0",
            "0",
            "0",
            "0",
            "1.7453",
            "arm_2_wrist_link",
            "mono_camera",
        ],
        name="mono_camera_tf_broadcaster",
        output="both",
    )

    return LaunchDescription(
        [
            aruco_detection_node,
            d435_aruco_detection_node,
            d435_camera_tf_broadcaster,
            mono_camera_tf_broadcaster,
        ]
    )
