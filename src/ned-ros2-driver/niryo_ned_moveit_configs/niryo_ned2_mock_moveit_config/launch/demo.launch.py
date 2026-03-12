from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    """
    Launch file for niryo_ned2 mock MoveIt demo with interactive RViz visualization.

    This uses MoveIt's generate_demo_launch which provides:
    - Mock joint system (no real hardware required)
    - Joint State Publisher GUI for manual joint control
    - Robot State Publisher (TF transforms)
    - MoveIt move_group (planning and execution)
    - RViz with interactive motion planning markers

    Note: The demo.launch.py uses default CHOMP planner. For OMPL planner with
    automatic trajectory timing, use test2.py which explicitly requests OMPL.

    Usage: ros2 launch niryo_ned2_mock_moveit_config demo.launch.py
    """
    moveit_config = MoveItConfigsBuilder(
        "niryo_ned2_mock", package_name="niryo_ned2_mock_moveit_config"
    ).to_moveit_configs()
    return generate_demo_launch(moveit_config)
