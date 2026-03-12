#!/usr/bin/env python3
"""
Load workspace planning scene from YAML file using ROS 2 publisher.

This script uses the standard ROS 2 planning scene API to load collision objects
from a YAML file and publish them to the planning scene.

Usage:
    python3 load_workspace_scene.py <path_to_yaml>

Or via launch file (automatically finds config/workspace_scene.yaml):
    ros2 launch niryo_ned2_mock_moveit_config sim_mock_ompl.launch.py
"""

import sys
import yaml
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


class SceneLoader(Node):
    """Load and publish planning scene objects."""

    def __init__(self):
        super().__init__("load_workspace_scene")

        print("\n" + "=" * 70)
        print("LOADING WORKSPACE PLANNING SCENE")
        print("=" * 70)

        # Determine scene file path
        if len(sys.argv) > 1:
            scene_file = sys.argv[1]
        else:
            # Default: look for config/workspace_scene.yaml
            from ament_index_python.packages import get_package_share_directory

            try:
                share_dir = get_package_share_directory("niryo_ned2_mock_moveit_config")
                scene_file = str(Path(share_dir) / "config" / "workspace_scene.yaml")
            except:
                # Fallback to source directory
                script_dir = Path(
                    __file__
                ).parent.parent  # src -> niryo_ned2_mock_moveit_config
                scene_file = str(script_dir / "config" / "workspace_scene.yaml")

        print(f"\nScene file: {scene_file}")

        # Load YAML
        try:
            with open(scene_file, "r") as f:
                scene_data = yaml.safe_load(f)
            print("✓ Loaded YAML file\n")
        except FileNotFoundError:
            self.get_logger().error(f"Scene file not found: {scene_file}")
            sys.exit(1)
        except yaml.YAMLError as e:
            self.get_logger().error(f"Failed to parse YAML: {e}")
            sys.exit(1)

        # Create publisher for planning scene
        self.scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 1)

        # Wait for subscribers
        time.sleep(0.5)

        # Parse and publish objects
        self.publish_scene(scene_data)

        print("=" * 70)
        print("✓ Planning scene published successfully!")
        print("=" * 70 + "\n")

    def publish_scene(self, scene_data: dict) -> None:
        """Parse YAML and publish collision objects to planning scene."""

        if not scene_data or "collision_objects" not in scene_data:
            print("WARNING: No collision_objects found in scene file")
            return

        # Create planning scene message
        planning_scene = PlanningScene()
        planning_scene.is_diff = True

        # Add objects
        print("Adding objects to planning scene:")
        for obj_data in scene_data.get("collision_objects", []):
            obj_id = obj_data.get("id")
            print(f"  - {obj_id}", end=" ")

            try:
                collision_obj = self.create_collision_object(obj_data)
                planning_scene.world.collision_objects.append(collision_obj)
                print("✓")
            except Exception as e:
                print(f"✗ Error: {e}")
                continue

        # Publish scene
        print("\nPublishing to /planning_scene...")
        self.scene_pub.publish(planning_scene)
        time.sleep(0.5)  # Give time for message to be delivered

    def create_collision_object(self, obj_data: dict) -> CollisionObject:
        """Create a CollisionObject from YAML data."""

        collision_obj = CollisionObject()

        # Set header
        collision_obj.header.frame_id = obj_data.get("header", {}).get(
            "frame_id", "world"
        )
        collision_obj.id = obj_data["id"]
        # Operation must be a byte: 0=ADD, 1=REMOVE, 2=APPEND, 3=MOVE
        collision_obj.operation = bytes([obj_data.get("operation", 0)])

        # Add primitives
        for prim_data in obj_data.get("primitives", []):
            primitive = SolidPrimitive()
            primitive.type = prim_data["type"]
            primitive.dimensions = prim_data["dimensions"]
            collision_obj.primitives.append(primitive)

        # Add poses
        for pose_data in obj_data.get("primitive_poses", []):
            pose = Pose()

            # Position
            pos = pose_data.get("position", {})
            pose.position.x = float(pos.get("x", 0.0))
            pose.position.y = float(pos.get("y", 0.0))
            pose.position.z = float(pos.get("z", 0.0))

            # Orientation
            ori = pose_data.get("orientation", {})
            pose.orientation.x = float(ori.get("x", 0.0))
            pose.orientation.y = float(ori.get("y", 0.0))
            pose.orientation.z = float(ori.get("z", 0.0))
            pose.orientation.w = float(ori.get("w", 1.0))

            collision_obj.primitive_poses.append(pose)

        return collision_obj


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    try:
        loader = SceneLoader()
        # Don't spin - just exit after publishing
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
