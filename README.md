# Husky A200 Autonomous Navigation (ROS 2 Jazzy + Gazebo Harmonic)

Custom mapless navigation stack (no Nav2) for Clearpath Husky A200 with VLP-16 lidar, using VFH+ reactive obstacle avoidance, BehaviorTree.CPP mission execution, and LLM-driven fleet management.

## Quick Start

```bash
source install/setup.bash
```

| # | Command | Purpose |
|---|---------|---------|
| 1 | `ros2 launch husky_bringup sim.launch.py` | Gazebo + robot spawn |
| 2 | `ros2 launch husky_bringup nav.launch.py namespace:=cpr_a200_0000` | Navigation stack |
| 3 | `ros2 launch husky_bringup rviz.launch.py namespace:=cpr_a200_0000` | Visualization |
| 4 | `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=True -r /cmd_vel:=/cpr_a200_0000/cmd_vel` | Keyboard control (real TTY) |
| 5 | `ros2 launch husky_bringup mission.launch.py namespace:=cpr_a200_0000 bt_file:=patrol_mission.xml` | BT autonomous mission |
| — | `ros2 launch husky_llm_bridge llm_bridge.launch.py` | LLM fleet pipeline |
| — | `husky` | LLM command CLI (after bridge) |

### Send a navigation goal

```bash
ros2 topic pub --once /cpr_a200_0000/goal_waypoints geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "odom"}, pose: {position: {x: 8.0, y: 5.0, z: 0.0}, orientation: {w: 1.0}}}'
```

## Architecture

### Nodes

| Node | Package | Purpose |
|------|---------|---------|
| `robot_state_publisher` | `robot_state_publisher` | Publishes joint transforms from URDF |
| `gz_ros_control` | `gz_ros2_control` | Bridge Gazebo physics → ros2_control |
| `joint_state_broadcaster` | `controller_manager` (spawner) | Publishes wheel joint states |
| `platform_velocity_controller` | `diff_drive_controller` (spawner) | Receives Twist → wheel joints |
| `vfh_planner` | `husky_nav` | VFH+ reactive planner → cmd_vel |
| `obstacle_detector` | `husky_nav` | Filters lidar point cloud → scan_2d |
| `stuck_detector` | `husky_nav` | Monitors cmd_vel vs actual velocity |
| `topic_health` | `husky_nav` | Diagnostic topic activity monitor |
| `ekf_gps` | `husky_nav` | GPS → odometry fusion |
| `mission_executor` | `husky_bt` | BehaviorTree.CPP mission executor |
| `fleet_manager` | `husky_fleet_manager` | Multi-robot state aggregation + goal dispatch |
| `llm_bridge` | `husky_llm_bridge` | Fleet-level LLM command bridge (Python) |
| `llm_validator` | `husky_llm_bridge` | JSON mission schema validator |
| `llm_connector` | `husky_llm_bridge` | LLM provider connector (Gemini or Ollama) |

### Topics (under `cpr_a200_0000/`)

**Input:**
| Topic | Type | Publisher |
|-------|------|-----------|
| `cmd_vel` | `TwistStamped` | teleop, vfh_planner |
| `goal_waypoints` | `PoseStamped` | mission_executor, user CLI |
| `rotation_goal` | `Float64` | llm_bridge (rotate action) |
| `emergency_stop` | `Bool` | hardware / fleet manager |

**Sensor:**
| Topic | Type | Publisher |
|-------|------|-----------|
| `velodyne_points` | `PointCloud2` | lidar bridge |
| `scan_2d` | `LaserScan` | obstacle_detector |

**State:**
| Topic | Type | Publisher |
|-------|------|-----------|
| `platform/odom` | `Odometry` | diff_drive controller |
| `platform/joint_states` | `JointState` | joint_state_broadcaster |
| `robot_state` | `RobotState` | mission_executor |
| `stuck` | `Bool` | stuck_detector |
| `topic_health` | `String` | topic_health_node |
| `vfh_goal_reached` | `Bool` | vfh_planner |

**Fleet (global):**
| Topic | Type | Publisher |
|-------|------|-----------|
| `/fleet/robot_states` | `FleetState` | fleet_manager |
| `/fleet/goal_events` | `GoalEvent` | fleet_manager |
| `/llm/raw_decision` | `String` | llm_connector |
| `/llm/decision` | `String` | llm_validator |
| `/llm/decision_status` | `String` | llm_validator |

### Data Flow

```
                         Gazebo World
                              │
                   gz_ros_control (plugin)
                              │
                   joint_state_broadcaster
                   platform_velocity_controller
                   ─→ /platform/odom
                   ←─ /platform/cmd_vel

  vfh_planner ←── scan_2d (from obstacle_detector)
  vfh_planner ←── goal_waypoints (from BT / user)
  vfh_planner ──→ cmd_vel
  vfh_planner ──→ vfh_goal_reached → BT feedback

  mission_executor (BT) ──→ goal_waypoints
                    ←── vfh_goal_reached, stuck, emergency_stop

  fleet_manager ←── robot_state (per robot)
               ──→ /fleet/robot_states
               ──→ navigate_to action (per robot)

  LLM pipeline: connector → validator → bridge → fleet_manager
```

## LLM Fleet Pipeline

### Launch

```bash
# DeepSeek (default)
export DEEPSEEK_API_KEY="your-key-here"
ros2 launch husky_llm_bridge llm_bridge.launch.py

# Gemini
ros2 launch husky_llm_bridge llm_bridge.launch.py connector:=gemini_connector_node.py

# Ollama (local, no key needed)
ros2 launch husky_llm_bridge llm_bridge.launch.py connector:=ollama_connector_node.py
```

Then use the CLI:
```bash
husky
```

### Pipeline stages

1. **Connector** — Calls LLM API (DeepSeek/Gemini/Ollama) with fleet state + robot odometry in prompt. Publishes raw JSON to `/llm/raw_decision`.
2. **Validator** — Validates JSON against mission schema (robot_id, action, waypoints, priority). Rejects malformed/invalid commands via `/llm/decision_status`.
3. **Bridge** — Converts validated decisions into ROS actions/topics per robot. Publishes waypoints, rotation goals, or state changes.
4. **Fleet Manager** — Aggregates per-robot states, dispatches goals via `navigate_to` action server, provides `/fleet/robot_states` and `/fleet/fleet_navigate`.

### JSON schema

```json
{"missions": [{
  "action": "navigate" | "set_state" | "rotate",
  "robot_id": "cpr_a200_0000",
  "waypoints": [{"x": 5.0, "y": 0.0, "yaw": 1.57}],
  "relative": {"forward": 2.0, "right": 0.5},
  "priority": 3,
  "angle_deg": 180.0
}]}
```

- `navigate`: Requires `waypoints[]` with x, y, optional yaw. Or `relative` for body-frame moves.
- `rotate`: Requires `angle_deg` (-360 to 360). Positive = CCW.
- `set_state`: Sets `waiting` or `avoidance_enabled` flags.
- Priority: 1 (lowest) to 5 (highest).

### Bypass LLM (test directly)

```bash
ros2 topic pub --once /llm/decision std_msgs/String \
  'data: "{\"missions\":[{\"action\":\"navigate\",\"robot_id\":\"cpr_a200_0000\",\"waypoints\":[{\"x\":2.0,\"y\":0.0}],\"priority\":3}]}"'
```

### Debug topics

| Command | Shows |
|---------|-------|
| `ros2 topic echo /llm/raw_decision` | Raw LLM output |
| `ros2 topic echo /llm/decision_status` | Validation result |
| `ros2 topic echo /fleet/goal_events` | Goal dispatch events |
| `ros2 topic echo /fleet/robot_states` | All robot states |
| `ros2 topic echo /cpr_a200_0000/robot_state` | Single robot state |

## Safety Features

### VFH+ Obstacle Avoidance

VFH+ (Vector Field Histogram) planner in `vfh_planner_node` provides reactive mapless navigation:

- Builds polar histogram (72 sectors × 5°) from live `scan_2d`
- Finds free valleys (contiguous free sectors, min gap width 0.3 rad)
- Selects valley via cost function: `|angle_to_goal| + 0.3 × (1/valley_width)`
- Scales speed by obstacle proximity (0.15–0.45 m/s)
- **Poles** (< 0.5 rad width): steers toward nearest gap at 50% speed
- **Walls** (≥ 0.5 rad width): stops completely
- Checks ±0.5 rad forward cone (not single ray)
- Visualizes path on `global_path` for RViz

Parameters in `config/pure_pursuit_params.yaml` under `vfh_planner:` section.

### Stuck Detection

| Parameter | Default | Description |
|-----------|---------|-------------|
| `speed_threshold` | 0.1 m/s | Min commanded speed to monitor |
| `stuck_threshold` | 0.05 m/s | Actual speed below this = stuck |
| `grace_period` | 2.0 s | Delay before monitoring starts |
| `stuck_timeout` | 8.0 s | Duration of stuck → recovery |

Behavior: RecoveryRotate (3s) → up to 3 attempts → permanent halt. Operator must send new goal.

### Emergency Stop

- Published to `/<robot_id>/emergency_stop` (Bool)
- Software-originated (LLM/fleet): auto-cleared on new goal
- Hardware-originated: must be explicitly cleared
- Pure pursuit/VFH node subscribes and zeros cmd_vel immediately

### GPS Fix Loss

BT node `GpsFixCheck` monitors `/sensors/gps_0/fix`. If `status.status == -1` or no message for 5s, tree halts. Resumes automatically when fix returns.

### Rotate Action

In-place rotation via LLM command. Publishes angle to `/<robot_id>/rotation_goal`. VFH planner enters rotation mode (linear.x = 0, angular.z = sign(angle) × 0.4 rad/s). Exits at 0.1 rad tolerance or 10s timeout.

## Robot Config

### `/root/clearpath/robot.yaml`

A200 URDF defines `top_chassis_link` (not `top_plate_link`). Sensor parent fields must match:

```yaml
sensors:
  lidar3d:
    - model: velodyne_lidar
      parent: top_chassis_link
      xyz: [0.0, 0.0, 0.12]
  gps:
    - model: garmin_18x
      parent: top_chassis_link
      xyz: [0.0, 0.0, 0.1]
```

Validate changes:
```bash
source install/setup.bash
ros2 run clearpath_generator_common generate_description -s /root/clearpath/
ros2 run clearpath_generator_common generate_semantic_description -s /root/clearpath/
ros2 run clearpath_generator_gz generate_launch -s /root/clearpath/
ros2 run clearpath_generator_gz generate_param -s /root/clearpath/
```

### Namespaces

All nodes run under `cpr_a200_0000/` namespace. Multi-robot support = different namespace.

## Repo Layout

```
src/
  husky_bringup/        — launch files, configs, worlds, RViz config
  husky_bt/             — BehaviorTree.CPP nodes + XML trees
  husky_nav/            — vfh_planner, obstacle_detector, stuck_detector, topic_health, ekf_gps (C++)
  husky_msgs/           — custom messages, services, actions
  husky_fleet_manager/  — fleet state aggregation + goal dispatch (C++)
  husky_llm_bridge/     — LLM pipeline: connector → validator → bridge (Python)
```

## Troubleshooting

### "parent link not found" in generation

A200 URDF defines `top_chassis_link`, not `top_plate_link`. Fix sensor `parent:` in `robot.yaml`.

### Controllers fail to configure

Spawners retry for 60s. Check:
```bash
ros2 control list_controllers
```
Expected: `joint_state_broadcaster` and `platform_velocity_controller` active.

### Action server not available

```
[WARN] [fleet_manager_node-1]: Action server not available for cpr_a200_0000
```
Ensure `mission.launch.py` (Terminal 5) is running before LLM commands.

### No scan_2d data

VFH planner waits 3s for scan data, then drives blind at reduced speed. Check obstacle_detector output:
```bash
ros2 topic echo /cpr_a200_0000/scan_2d
```

### Teleop fails with termios.error

Run in a real terminal (not headless/script).

### Odometry topic

All nodes subscribe to `platform/odom` (diff_drive controller). EKF-filtered `platform/odom/filtered` only exists when GPS/IMU fusion is explicitly launched.
