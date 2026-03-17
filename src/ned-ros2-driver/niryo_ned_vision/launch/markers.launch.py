from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch file for all ArUco markers detection and publishing as TF transforms."""

    # All markers publisher node for first camera (camera_id=0)
    markers_publisher_node = Node(
        package="niryo_ned_vision",
        executable="marker_publisher",
        name="markers_publisher",
        output="both",
        parameters=[
            {"camera_id": 0},
            {"camera_frame_id": "camera_1"},
            {"enable_visualization": False},
        ],
    )

    # All markers publisher node for second camera (camera_id=4)
    markers_publisher_node2 = Node(
        package="niryo_ned_vision",
        executable="marker_publisher",
        name="markers_publisher2",
        output="both",
        parameters=[
            {"camera_id": 4},
            {"camera_frame_id": "camera_2"},
            {"enable_visualization": False},
        ],
    )

    # Static transform publisher for D435 camera
    camera1_tf_broadcaster = Node(
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
            "camera_1",
        ],
        name="camera1_tf_broadcaster",
        output="both",
    )

    # Static transform publisher for mono camera
    camera2_tf_broadcaster = Node(
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
            "camera_2",
        ],
        name="camera2_tf_broadcaster",
        output="both",
    )

    # Static transform publisher for ArUco marker reference frame
    aruco_marker_tf_broadcaster = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "world",
            "aruco_marker_40",
        ],
        name="aruco_marker_tf_broadcaster",
        output="both",
    )

    return LaunchDescription(
        [
            markers_publisher_node,
            markers_publisher_node2,
            camera1_tf_broadcaster,
            camera2_tf_broadcaster,
            aruco_marker_tf_broadcaster,
        ]
    )
