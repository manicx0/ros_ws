# Husky A200 Autonomous Navigation (ROS 2 Jazzy + Gazebo Harmonic)

Custom navigation stack (no Nav2) for Clearpath Husky A200 with VLP-16 lidar, using pure pursuit, obstacle detection, path planning, and BehaviorTree.CPP mission execution.

## Architecture

### Nodes

| Node | Package | Purpose |
|------|---------|---------|
| `robot_state_publisher` | `robot_state_publisher` | Publishes joint transforms from URDF |
| `gz_ros_control` | `gz_ros2_control` (Gazebo plugin) | Bridge Gazebo physics → ros2_control |
| `controller_manager` | `controller_manager` (loaded by gz_ros_control) | Manages lifecycle of controllers |
| `joint_state_broadcaster` | `controller_manager` (spawner) | Publishes wheel joint positions/velocities |
| `platform_velocity_controller` | `diff_drive_controller` (spawner) | Receives Twist → applies to wheel joints |
| `ekf_node` | `robot_localization` | Fuses wheel odom + IMU into filtered odom |
| `lidar3d_0_gz_bridge` | `ros_gz_bridge` | Bridges Gazebo lidar scan → ROS topics |
| `cmd_vel_bridge` | `ros_gz_bridge` | Bridges ROS cmd_vel → Gazebo Twist |
| `odom_base_tf_bridge` | `ros_gz_bridge` | Bridges Gazebo TF → ROS tf |
| `pure_pursuit` | `husky_nav` | Follows planned path via cmd_vel |
| `path_planner` | `husky_nav` | Plans path from current pose to goal |
| `obstacle_detector` | `husky_nav` | Detects obstacles in lidar point cloud |
| `ekf_gps` | `husky_nav` | GPS → odometry fusion (expansion) |
| `goal_pose_relay` | `husky_nav` | Relays RViz 2D Nav Goal → goal_waypoints |
| `mission_executor` | `husky_bt` | Runs behavior tree XML for autonomous patrol |
| `twist_mux` | `twist_mux` | Multiplexes cmd_vel from multiple sources |
| `teleop_twist_joy` | `teleop_twist_joy` | Joystick → cmd_vel |

### Topics (under `cpr_a200_0000/`)

**Input:**
| Topic | Type | Publisher |
|-------|------|-----------|
| `cmd_vel` | `TwistStamped` | teleop_keyboard, pure_pursuit, twist_marker_server |
| `goal_waypoints` | `PoseStamped` | mission_executor, goal_pose_relay, user CLI |
| `/goal_pose` | `PoseStamped` | RViz 2D Nav Goal tool (global, not namespaced) |

**Sensor:**
| Topic | Type | Publisher |
|-------|------|-----------|
| `velodyne_points` | `PointCloud2` | lidar bridge (3D full) |
| `scan_2d` | `LaserScan` | lidar bridge (2D projection) |
| `sensors/lidar3d_0/scan` | `LaserScan` | lidar bridge (raw) |
| `sensors/lidar3d_0/points` | `PointCloud2` | lidar bridge (raw) |

**State / Odometry:**
| Topic | Type | Publisher |
|-------|------|-----------|
| `platform/joint_states` | `JointState` | joint_state_broadcaster |
| `platform/odom` | `Odometry` | platform_velocity_controller |
| `odometry/filtered` | `Odometry` | ekf_node |
| `tf` | `TFMessage` | robot_state_publisher + ekf_node |
| `tf_static` | `TFMessage` | robot_state_publisher |

**Plan:**
| Topic | Type | Publisher |
|-------|------|-----------|
| `global_path` | `Path` | path_planner_node |

**Processed:**
| Topic | Type | Publisher |
|-------|------|-----------|
| `filtered_cloud` | `PointCloud2` | obstacle_detector_node |

### Data Flow

```
                    Gazebo World
                         │
              gz_ros_control (plugin in URDF)
                         │
                 controller_manager
                    │     │      │
       joint_state  │     │      │  platform_velocity_controller
       _broadcaster │     │      │  ─→ /platform/odom
       ─→ /platform/│     │      │  ←─ /platform/cmd_vel
         joint_states│     │      │
                    │     │      │
              robot_state  cmd_vel_bridge ←── twist_mux ←── pure_pursuit
              _publisher  (ROS↔Gazebo)              ↑      path_planner
              ─→ /tf                              teleop    obstacle_detector
                                                    │
                                             mission_executor (BT)
                                             ─→ /goal_waypoints

                    ekf_node ←── /platform/odom
                    ─→ /odometry/filtered
                    ─→ odom→base_link (tf)

                    obstacle_detector ←── /velodyne_points
                    ─→ /filtered_cloud
```

### Repo Layout

```
src/
  clearpath_common/   — upstream (gitignored, installed at /opt/ros/jazzy)
  clearpath_gz/        — upstream (gitignored, installed at /opt/ros/jazzy)
  husky_bringup/       — launch files, configs, worlds, RViz config
  husky_bt/            — BehaviorTree.CPP mission executor + BT node plugins
  husky_nav/           — pure_pursuit, path_planner, obstacle_detector, ekf_gps (C++)
```

All C++ topic strings use relative paths (no leading `/`) to inherit namespace from `PushRosNamespace`.

## Launch Commands

All commands require `source install/setup.bash` first.

| Terminal | Command | Purpose |
|----------|---------|---------|
| 1 | `ros2 launch husky_bringup sim.launch.py` | Gazebo simulation + robot spawn |
| 2 | `ros2 launch husky_bringup nav.launch.py namespace:=cpr_a200_0000` | Navigation stack (pure pursuit, path planner, obstacle detector, EKF) |
| 3 | `ros2 launch husky_bringup rviz.launch.py namespace:=cpr_a200_0000` | RViz visualization |
| 4 | `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=True -r /cmd_vel:=/cpr_a200_0000/cmd_vel` | Manual keyboard control |
| 5 | `ros2 launch husky_bringup mission.launch.py namespace:=cpr_a200_0000 bt_file:=patrol_mission.xml` | Autonomous BT mission |

**LLM Fleet Pipeline** (alternative to terminals 4-5):

| Terminal | Command | Purpose |
|----------|---------|---------|
| 4 | `ros2 launch husky_llm_bridge llm_bridge.launch.py` | Fleet manager + LLM bridge |
| 5 | `husky` | Interactive LLM command CLI |

## Quick Start

### Prerequisites

Robot config at `/root/clearpath/robot.yaml`:

```yaml
system:
  username: root
  namespace: cpr_a200_0000

platform:
  model: a200

sensors:
  lidar3d:
    - model: velodyne_lidar
      urdf_enabled: true
      launch_enabled: true
      parent: top_chassis_link    # NOT top_plate_link (doesn't exist in A200 URDF)
      xyz: [0.0, 0.0, 0.12]
      rpy: [0.0, 0.0, 0.0]
```

All commands need `source install/setup.bash` first.

### Terminal 1: Gazebo + Robot

```bash
source install/setup.bash
ros2 launch husky_bringup sim.launch.py
```

Wait for robot entity in Gazebo. Controllers self-configure within ~30s.

### Terminal 2: Navigation stack

```bash
source install/setup.bash
ros2 launch husky_bringup nav.launch.py namespace:=cpr_a200_0000
```

Starts pure_pursuit, path_planner, obstacle_detector, EKF.

### Terminal 3: RViz

```bash
source install/setup.bash
ros2 launch husky_bringup rviz.launch.py namespace:=cpr_a200_0000
```

Robot model, TF, point cloud, path, odometry. Separate file so RViz doesn't die when sim restarts.

### Terminal 4: Teleop (requires real TTY)

```bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p stamped:=True -r /cmd_vel:=/cpr_a200_0000/cmd_vel
```

### Terminal 5: Autonomous goal

**Option A: RViz 2D Nav Goal (recommended)**

In RViz, click the **"2D Nav Goal"** button in the toolbar, then click anywhere in the scene. The robot drives there. The `goal_pose_relay` node bridges RViz's `/goal_pose` topic to the navigation stack's `goal_waypoints` topic.

**Option B: CLI command**

```bash
source install/setup.bash
ros2 topic pub --once /cpr_a200_0000/goal_waypoints geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "odom"}, pose: {position: {x: 8.0, y: 5.0, z: 0.0}, orientation: {w: 1.0}}}'
```

### Terminal 5: BehaviorTree mission

```bash
source install/setup.bash
ros2 launch husky_bringup mission.launch.py namespace:=cpr_a200_0000 bt_file:=patrol_mission.xml
```

### Terminal 6: LLM Fleet Pipeline

```bash
export GEMINI_API_KEY="your-key-here"
source install/setup.bash
ros2 launch husky_llm_bridge llm_bridge.launch.py
```

Then use the CLI to send commands:

```bash
husky
```

Interactive REPL:
```
husky> send robot to x=3,y=0
[Sent] send robot to x=3,y=0

husky> status
cpr_a200_0000: IDLE

husky> help
Commands:
  <text>    Send command to LLM (e.g., "send robot to x=3,y=0")
  status    Show current fleet state
  help      Show this help
  clear     Clear screen
  quit      Exit CLI

husky> quit
```

### Autonomous goal

Stop the current goal and regain manual control:

```bash
# 1. Cancel the goal (keeps all nodes running)
ros2 topic pub --once /cpr_a200_0000/goal_waypoints geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "odom"}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'

# 2. Zero out residual velocity
ros2 topic pub --once /cpr_a200_0000/cmd_vel geometry_msgs/msg/TwistStamped \
  '{twist: {linear: {x: 0.0}, angular: {z: 0.0}}}'
```

Teleop keyboard (Terminal 4) now has priority via twist_mux. Alternatively, Ctrl+C the mission executor (Terminal 5) to stop autonomous goals entirely.

## Namespaces

All nodes run under `cpr_a200_0000/` namespace. Topics are scoped to the robot, enabling future multi-robot use — adding a second Husky is just a different namespace.

## LLM Fleet Pipeline

### Prerequisites

Set API key before launching the bridge:

```bash
export GEMINI_API_KEY="your-key-here"
```

Or add to `~/.bashrc` for persistence:

```bash
echo 'export GEMINI_API_KEY="your-key-here"' >> ~/.bashrc
```

### Debugging

Run in any terminal after sending a command:

| Check | Command |
|-------|---------|
| Raw Gemini output | `ros2 topic echo /llm/raw_decision` |
| Validation result | `ros2 topic echo /llm/decision_status` |
| Goal events | `ros2 topic echo /fleet/goal_events` |
| Fleet states | `ros2 topic echo /fleet/robot_states` |
| Robot state | `ros2 topic echo /cpr_a200_0000/robot_state` |
| Velocity commands | `ros2 topic echo /cpr_a200_0000/cmd_vel` |

### Bypass LLM (test bridge directly)

```bash
ros2 topic pub --once /llm/decision std_msgs/String \
  'data: "{\"missions\":[{\"action\":\"navigate\",\"robot_id\":\"cpr_a200_0000\",\"waypoints\":[{\"x\":2.0,\"y\":0.0}],\"priority\":3}]}"'
```

### Common Issues

#### No API Key

**Symptom:**
```
[WARN] [gemini_connector_node.py-4]: Status: No GEMINI_API_KEY configured
```

**Solution:**
```bash
export GEMINI_API_KEY="your-key"
# Then restart the LLM bridge terminal
```

#### Action Server Not Available

**Symptom:**
```
[WARN] [fleet_manager_node-1]: Action server not available for cpr_a200_0000
```

**Cause:** The mission executor (Terminal 5) isn't running or hasn't started yet.

**Solution:** Ensure Terminal 5 is running before sending commands.

#### Validation Errors

The validator rejects invalid data. Examples:

**Invalid robot_id:**
```
[llm_validator_node.py-3]: Validation failed: Mission[0]: robot_id "invalid_robot" not in fleet config
```

**Missing waypoints:**
```
[llm_validator_node.py-3]: Validation failed: Mission[0]: navigate action must have waypoints
```

**Invalid priority:**
```
[llm_validator_node.py-3]: Validation failed: Mission[0]: priority must be 1-5
```

## Troubleshooting

### Robot not visible in Gazebo

Check generation pipeline:

```bash
source install/setup.bash
ros2 run clearpath_generator_common generate_description -s /root/clearpath/
ros2 run clearpath_generator_common generate_semantic_description -s /root/clearpath/  # catches missing links
ros2 run clearpath_generator_gz generate_launch -s /root/clearpath/
ros2 run clearpath_generator_gz generate_param -s /root/clearpath/
```

If "parent link not found" → A200 URDF defines `top_chassis_link`, not `top_plate_link`.

### Controllers fail to configure

Spawners retry for 60s. Check:

```bash
ros2 control list_controllers
ros2 control list_hardware_components
```

Normal output:
```
[spawner-*] Configured and activated joint_state_broadcaster
[spawner-*] Configured and activated platform_velocity_controller
```

### Teleop crashes with `termios.error`

Non-TTY environment — run in a real terminal window, not headless/script.