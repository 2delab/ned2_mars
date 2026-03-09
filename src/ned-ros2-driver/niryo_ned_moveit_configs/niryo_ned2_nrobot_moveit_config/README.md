# Niryo Ned2 N-Robot MoveIt Configuration

**Scalable MoveIt2 configuration for Niryo Ned2 robotic arms - support for arbitrary number of robots positioned around a central workspace.**

## Overview

This package provides a fully parametric, configuration-driven approach to managing multiple Niryo Ned2 robots as a unified MoveIt2 system. Instead of creating separate packages for dual-arm, triple-arm, etc., you define all robots in a single `robot_config.yaml` file, and all other configurations are automatically generated.

### Key Features

- **Single Source of Truth**: All robot definitions in `config/robot_config.yaml`
- **Automatic Generation**: SRDF, controllers, kinematics, and limits auto-generated from config
- **Arbitrary Scaling**: Add/remove robots by editing YAML (no code changes needed)
- **Flexible Positioning**: Define robots at any angle and distance from origin
- **Independent Package**: Fully self-contained, doesn't modify existing dual-arm package
- **Fake System Included**: Ready for testing/simulation without hardware

## Quick Start

### 1. View Current Configuration

The default configuration defines 3 robots at 120° spacing:

```bash
cat config/robot_config.yaml
```

Current robots:
- `arm_1`: 180° (pointing backward)
- `arm_2`: 0° (pointing forward) 
- `arm_3`: 120° (120° from arm_2)

All positioned 35cm from the world origin.

### 2. Generate Configurations

After editing `robot_config.yaml`, regenerate all dependent configs:

```bash
python3 scripts/generate_all.py
```

This creates:
- `niryo_ned2_nrobot.srdf` - Semantic descriptions, groups, collision rules
- `ros2_controllers.yaml` - ROS2 control framework config
- `moveit_controllers.yaml` - MoveIt trajectory execution config
- `kinematics.yaml` - Inverse kinematics solvers
- `joint_limits.yaml` - Joint velocity/acceleration limits
- `initial_positions.yaml` - ROS2 control fake system initial positions

### 3. Launch the System

```bash
# Demo mode (no hardware required)
ros2 launch niryo_ned2_nrobot_moveit_config demo_nrobot.launch.py
```

This starts:
- Joint state prefixer (aggregates N robot states)
- Robot state publisher (publishes TF frames)
- MoveIt move_group (planning and execution)
- RViz visualization

## Modifying Robot Configuration

### Add a 4th Robot

Edit `config/robot_config.yaml`:

```yaml
robots:
  - name: arm_1
    angle_deg: 180.0
    distance_m: 0.35
  
  - name: arm_2
    angle_deg: 0.0
    distance_m: 0.35
  
  - name: arm_3
    angle_deg: 120.0
    distance_m: 0.35
  
  - name: arm_4
    angle_deg: 240.0          # New robot at 240°
    distance_m: 0.35
```

Then regenerate:

```bash
python3 scripts/generate_all.py
```

The system automatically scales to 4 robots - no code changes needed.

### Custom Robot Positioning

Robots are positioned using polar coordinates (angle, distance from origin):

```yaml
robots:
  - name: arm_center
    angle_deg: 0.0            # 0° = -X axis (forward)
    distance_m: 0.0           # At origin
  
  - name: arm_far
    angle_deg: 45.0           # 45° from +X axis
    distance_m: 0.5           # 50cm from origin
```

**Angle reference**:
- 0° = -X axis (forward)
- 90° = +Y axis (left)
- 180° = +X axis (backward)
- 270° = -Y axis (right)

## Configuration Files

### `robot_config.yaml` (PRIMARY SOURCE OF TRUTH)

Defines all robots, groups, and default poses. Edit this file to scale the system.

**Example**:
```yaml
robots:
  - name: arm_1
    angle_deg: 180.0
    distance_m: 0.35

groups:
  all: [arm_1, arm_2, arm_3]
  pair: [arm_1, arm_2]

default_home_pose:
  joint_1: 0.0
  joint_2: 0.2862
  # ... etc
```

### Generated Configuration Files

**DO NOT EDIT** these directly - they're regenerated from `robot_config.yaml`:

- **`niryo_ned2_nrobot.srdf`** - Semantic robot description (groups, collisions, states)
- **`ros2_controllers.yaml`** - ROS2 control system configuration
- **`moveit_controllers.yaml`** - MoveIt trajectory execution routing
- **`kinematics.yaml`** - KDL inverse kinematics solver config
- **`joint_limits.yaml`** - Joint velocity/acceleration limits
- **`initial_positions.yaml`** - Initial joint positions for fake system

### Static Files

- **`niryo_ned2_nrobot.urdf.xacro`** - Parametric URDF (loads robot_config.yaml)
- **`niryo_ned2_nrobot.ros2_control.xacro`** - ROS2 control hardware interface macro
- **`moveit.rviz`** - RViz configuration
- **`pilz_cartesian_limits.yaml`** - Cartesian motion limits

## System Architecture

```
robot_config.yaml (single definition point)
    ↓
generate_all.py (orchestrator)
    ├→ generate_srdf.py
    ├→ generate_controllers.py
    ├→ generate_kinematics.py
    ├→ generate_joint_limits.py
    └→ generate_initial_positions.py
    ↓
Generated YAML configs (SRDF, controllers, kinematics, limits)
    ↓
nrobot_moveit_launch.py
    ├→ Joint State Prefixer (aggregates N robot states)
    ├→ Robot State Publisher (publishes TF)
    ├→ MoveIt move_group (planning/execution)
    └→ RViz visualization
```

## Scripts

### `generate_all.py`

Orchestrator script - runs all generators in sequence with error handling.

```bash
python3 scripts/generate_all.py
```

**Output**:
- Generates all config files
- Reports success/failure
- Prints next steps

### Individual Generators

Generate specific config files:

```bash
python3 scripts/generate_srdf.py          # → niryo_ned2_nrobot.srdf
python3 scripts/generate_controllers.py   # → ros2_controllers.yaml, moveit_controllers.yaml
python3 scripts/generate_kinematics.py    # → kinematics.yaml
python3 scripts/generate_joint_limits.py  # → joint_limits.yaml
python3 scripts/generate_initial_positions.py  # → initial_positions.yaml
```

## Launch Files

### `nrobot_moveit_launch.py` (Main Entry Point)

Full MoveIt2 system launch:

```bash
ros2 launch niryo_ned2_nrobot_moveit_config nrobot_moveit_launch.py
```

**Includes**:
- Joint state prefixer (dynamic robot list from robot_config.yaml)
- Robot state publisher
- MoveIt move_group
- RViz visualization

**Optional arguments**:
```bash
# Use custom RViz config
ros2 launch niryo_ned2_nrobot_moveit_config nrobot_moveit_launch.py rviz_config:=custom.rviz

# Use custom robot config file (for different scenarios)
ros2 launch niryo_ned2_nrobot_moveit_config nrobot_moveit_launch.py robot_config_file:=config/my_config.yaml
```

### `demo_nrobot.launch.py` (Demo Mode)

Simplified launch for testing - includes fake ROS2 control system:

```bash
ros2 launch niryo_ned2_nrobot_moveit_config demo_nrobot.launch.py
```

**Usage**: Test without hardware or driver nodes.

## Key Nodes

### Joint State Prefixer (`src/joint_state_prefixer.py`)

**Purpose**: Bridges multiple robot drivers to unified MoveIt system

**Subscribes to**:
- `/arm_X/joint_states` (from each driver, unprefixed joint names)

**Publishes**:
- `/joint_states` (aggregated, with prefixed joint names: `arm_1_joint_1`, `arm_2_joint_1`, etc.)
- Proxy action servers for trajectory execution

**Dynamic scaling**: Automatically handles any number of robots via `robot_namespaces` parameter

**Executor optimization**: Thread pool size scales with number of robots

## Testing & Validation

### Phase 1: Verify 2-Robot Configuration

Reset config to 2 robots (edit `robot_config.yaml`, keep only arm_1 and arm_2):

```bash
python3 scripts/generate_all.py
ros2 launch niryo_ned2_nrobot_moveit_config demo_nrobot.launch.py
```

In RViz, verify:
- Both robots visible
- ~180° apart positioning
- All TF frames present (`arm_1_base_link`, `arm_2_base_link`, etc.)
- Joint state topics active

### Phase 2: Verify 3-Robot Configuration (120° spacing)

Enable arm_3 in `robot_config.yaml`:

```bash
python3 scripts/generate_all.py
ros2 launch niryo_ned2_nrobot_moveit_config demo_nrobot.launch.py
```

In RViz, verify:
- All 3 robots visible
- ~120° spacing
- All joint/TF topics present
- Planning works for individual arms and groups

### Phase 3: Test Motion Planning

```bash
# In another terminal
ros2 run moveit_ros_move_group move_group &

# Plan motion for arm_1
# Use RViz Motion Planning tab or write custom planner
```

## Troubleshooting

### Generation Script Fails

Check robot_config.yaml syntax:
```bash
python3 -c "import yaml; yaml.safe_load(open('config/robot_config.yaml'))"
```

### Robots Don't Appear in RViz

1. Check joint state prefixer is running:
   ```bash
   ros2 topic list | grep joint_states
   ros2 topic echo /joint_states
   ```

2. Check TF frames are being published:
   ```bash
   ros2 run tf2_tools view_frames
   ```

3. Check robot state publisher is publishing:
   ```bash
   ros2 topic list | grep robot_description
   ```

### MoveIt Planning Fails

1. Check move_group is running:
   ```bash
   ros2 node list | grep move_group
   ```

2. Check controllers are active:
   ```bash
   ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers
   ```

3. Check SRDF is valid:
   ```bash
   xmllint config/niryo_ned2_nrobot.srdf
   ```

## Advanced Usage

### Using Subsets of Robots

Define planning groups in `robot_config.yaml`:

```yaml
groups:
  all: [arm_1, arm_2, arm_3]
  front_pair: [arm_1, arm_2]  # Left and right
  circle: [arm_1, arm_2, arm_3]  # All
```

Then plan specifically for each group using MoveIt's group_name parameter.

### Custom Default Poses

Edit `default_home_pose` in `robot_config.yaml`:

```yaml
default_home_pose:
  joint_1: 0.0
  joint_2: 0.5      # More arm lift
  joint_3: 0.0
  joint_4: 1.57     # ~90° rotation
  joint_5: -1.5166
  joint_6: 0.0
```

Then regenerate configs - all robots will use this pose.

### Multiple Configuration Variants

Create separate config files:

```bash
cp config/robot_config.yaml config/robot_config_2arm.yaml
cp config/robot_config.yaml config/robot_config_3arm.yaml
cp config/robot_config.yaml config/robot_config_4arm.yaml
```

Edit each variant and regenerate as needed.

## File Structure

```
niryo_ned2_nrobot_moveit_config/
├── config/
│   ├── robot_config.yaml               ← EDIT THIS (primary source)
│   ├── niryo_ned2_nrobot.urdf.xacro    (parametric)
│   ├── niryo_ned2_nrobot.ros2_control.xacro
│   ├── niryo_ned2_nrobot.srdf          (generated)
│   ├── ros2_controllers.yaml           (generated)
│   ├── moveit_controllers.yaml         (generated)
│   ├── kinematics.yaml                 (generated)
│   ├── joint_limits.yaml               (generated)
│   ├── initial_positions.yaml          (generated)
│   ├── moveit.rviz
│   └── pilz_cartesian_limits.yaml
├── src/
│   └── joint_state_prefixer.py         (N-robot aggregator)
├── launch/
│   ├── nrobot_moveit_launch.py         (main)
│   └── demo_nrobot.launch.py           (demo)
├── scripts/
│   ├── generate_all.py                 (orchestrator)
│   ├── generate_srdf.py
│   ├── generate_controllers.py
│   ├── generate_kinematics.py
│   ├── generate_joint_limits.py
│   └── generate_initial_positions.py
├── CMakeLists.txt
├── package.xml
├── .setup_assistant
└── README.md
```

## Development Notes

### Adding Support for More Robots

The system is designed to scale to any number of robots. Tested configurations:
- 2 robots ✓
- 3 robots ✓
- N robots (architecture supports arbitrary N)

### Generator Architecture

Each generator script:
1. Loads `robot_config.yaml`
2. Validates robot definitions
3. Generates appropriate YAML
4. Writes to config directory

Generators are independent and can be run individually or via orchestrator.

### URDF Parametrization

The URDF uses xacro's `load_yaml()` and `foreach` to:
1. Load `robot_config.yaml` 
2. Loop through each robot entry
3. Calculate cartesian coordinates from polar (angle, distance)
4. Instantiate `ned2_arm_assembly_nrobot` macro for each robot

This approach requires no Python processing during URDF expansion.

## Contributing

To modify the generation system:

1. **Add a new configuration parameter**: Edit `robot_config.yaml` schema
2. **Add a new generator**: Create `scripts/generate_newfeature.py`, add to `generate_all.py`
3. **Update templates**: Modify generator logic to produce desired output
4. **Test**: Run `python3 scripts/generate_all.py` and verify output

## License

BSD-3-Clause

## References

- [MoveIt2 Documentation](http://moveit.ros.org/)
- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [Niryo Ned2 Documentation](https://docs.niryo.com/)
