#!/usr/bin/env python3


import sys
import subprocess
from pathlib import Path


def run_generator(script_path, script_name):
    """Run a generator script and report results"""
    try:
        result = subprocess.run(
            [sys.executable, str(script_path)],
            capture_output=True,
            text=True,
            check=True,
        )
        print(result.stdout)
        if result.stderr:
            print(f"  Warning: {result.stderr}")
        return True
    except subprocess.CalledProcessError as e:
        print(f"✗ FAILED: {script_name}")
        print(f"  Error: {e.stderr}")
        return False


def main():
    """Main entry point - run all generators"""
    script_dir = Path(__file__).parent

    print("=" * 70)
    print("Generating MoveIt Configuration from robot_config.yaml")
    print("=" * 70)
    print()

    generators = [
        (script_dir / "generate_urdf_macro.py", "URDF macro generator"),
        (script_dir / "generate_ros2_control.py", "ROS2 control generator"),
        (script_dir / "generate_srdf.py", "SRDF generator"),
        (script_dir / "generate_controllers.py", "Controller generator"),
        (script_dir / "generate_kinematics.py", "Kinematics generator"),
        (script_dir / "generate_joint_limits.py", "Joint limits generator"),
        (script_dir / "generate_initial_positions.py", "Initial positions generator"),
    ]

    failed = []

    for script_path, script_name in generators:
        if not script_path.exists():
            print(f"✗ NOT FOUND: {script_name}")
            print(f"  {script_path}")
            failed.append(script_name)
            continue

        if not run_generator(script_path, script_name):
            failed.append(script_name)
        print()

    print("=" * 70)
    if failed:
        print(f"✗ Generation FAILED: {len(failed)} error(s)")
        for name in failed:
            print(f"  - {name}")
        return 1
    else:
        print("✓ All configurations generated successfully!")
        print()
        print("Next steps:")
        print("  1. Review generated files in config/")
        print(
            "  2. Launch the system: ros2 launch niryo_ned2_nrobot_moveit_config demo_nrobot.launch.py"
        )
        print()
        return 0


if __name__ == "__main__":
    sys.exit(main())
