# Husky A200 Autonomous Navigation (ROS 2 Jazzy + Gazebo Harmonic)

Custom mapless navigation stack (no Nav2) for Clearpath Husky A200 with VLP-16 lidar, using VFH+ reactive obstacle avoidance, BehaviorTree.CPP mission execution, and LLM-driven fleet management.

## Project Overview

A multi-robot Husky A200 fleet running in Gazebo with an LLM-driven natural language interface. An operator types commands like *"patrol point A, B, C"* or *"have robot 2 check the north perimeter"*, an LLM (DeepSeek/Gemini/Ollama) translates that into structured JSON missions, and a fleet manager dispatches goals to the right robots. The robots navigate outdoor terrain using VFH+ reactive obstacle avoidance with no pre-built map.

Built for a thesis demo — a 5–10 minute pipeline: human speaks → LLM plans → fleet assigns → robots execute → obstacles avoided → results reported back.

## Development Phases

### Phase 1 — Core Navigation Stack
Built the foundational single-robot stack on top of Clearpath's Gazebo simulation.
- `vfh_planner_node` — VFH+ mapless reactive obstacle avoidance (replaced path_planner + pure_pursuit)
- `obstacle_detector_node` — 3D lidar → 2D scan projection
- `stuck_detector_node` — velocity monitoring with recovery rotation
- `topic_health_node` — diagnostic topic monitor
- `ekf_gps_node` — GPS/Navsat + wheel odom + IMU fusion
- GPS waypoint navigation with named waypoints (`waypoints.yaml`)

### Phase 2 — Behavior Tree Architecture
State-aware BT executor with custom nodes for autonomous mission execution.
- `husky_bt` package with BehaviorTree.CPP-based mission executor
- Custom BT nodes: `NavigateToGoal`, `ObstacleCheck`, `RecoveryRotate`, `GpsFixCheck`, `IdleMonitor`, `EmergencyStopCondition`, `WaitingCondition`
- BT XML trees: `navigate_to_goal.xml`, `patrol_mission.xml`
- State flags on blackboard: `emergency_stop`, `waiting`, `idle`, `obstacle_detected`, `mission_active`, `has_goal`
- `SetRobotState` service for external state control

### Phase 3 — Message Definitions & Fleet Manager
Multi-robot message protocol and fleet-level coordination.
- `husky_msgs`: RobotState, GoalEvent, FleetState, FleetGoal, FleetResult messages; SetRobotState, FleetSetState services; NavigateTo, FleetNavigate actions
- `husky_fleet_manager`: C++ node aggregating per-robot states, dispatching goals via `FleetNavigate` action, batch state control via service
- Per-robot interfaces: `robot_state` topic, `navigate_to` action, `set_robot_state` service

### Phase 4 — LLM Pipeline
Full natural-language → robot command pipeline with multiple provider support.
- **3-node pipe**: connector (DeepSeek/Gemini/Ollama) → JSON validator → bridge
- Validator enforces schema: missions array, valid actions, valid robot_ids, required fields
- Sequential waypoints, named location resolution, yaw in waypoints, relative body-frame moves
- CLI tool (`husky`) — interactive REPL that talks to the LLM pipeline
- Fleet config in `fleet.yaml` with valid robot IDs injected into prompt
- Patrol with contingencies (on_obstacle: stop/return_home/skip_and_continue; on_stuck: abort/rotate_and_continue)

### Phase 5 — Multi-Robot Simulation
3-robot fleet infrastructure in a shared Gazebo world.
- 3 robot configs at `/root/clearpath/`, `/root/clearpath1/`, `/root/clearpath2/` (x=0, x=3, x=6)
- `multi_sim.launch.py` spawning 3 Huskies in `solar_farm` world
- `fleet.yaml` with 3 robots and home poses
- Per-robot LLM bridge publishers (rotation, nav goals)
- Odometry-based robot position tracking in fleet state

### Phase 6 — Bug Fixes & Polish
11+ bugs fixed across emergency stop, odometry, dedup, and VFH edge cases.
- Emergency stop actually stops robot (was only halting BT, not VFH planner)
- Post-emergency-stop goal clearing (hardware vs software source tracking)
- LLM connector dedup to prevent repeated command spam
- Robot odometry in LLM prompt so it can compute waypoints
- VFH scan timeout fallback (blind driving if no scan data)
- Fleet manager empty-goal abort, connector cache clearing on mission completion

## Current Status

### Done
- Single-robot VFH navigation
- GPS waypoint system
- Behavior tree executor
- LLM pipeline (3 providers)
- CLI interface
- Patrol with contingencies
- Emergency stop (full stack)
- Obstacle avoidance (VFH+)
- Stuck detection + recovery

### Remaining
- **Fleet manager** — Dispatch and state aggregation code exists but still under testing, not working reliably yet
- **Multi-robot simulation** — 3-robot configs and spawn exist but still under testing, not verified end-to-end
- **Fleet manager auto-reassignment** — Dispatches goals but doesn't reassign on failure
- **LLM fleet-level commands** — LLM knows individual robots but can't reason about fleet (e.g., "send nearest idle robot")
- **Multi-robot collision avoidance** — No inter-robot path coordination
- **Real hardware prep** — EKF retuning, real GPS driver, outdoor VFH tuning

### Thesis Demo Requirements
1. Natural language command → LLM plans mission ✅ (single robot)
2. Fleet manager assigns robots based on availability ⚠️ (dispatches but no smart selection)
3. Robots navigate GPS waypoints autonomously ✅ (single robot)
4. Obstacle encountered → avoided without intervention ✅
5. Robot completes mission → reports back → ready ✅
6. One robot fails → reassigned automatically ❌

## Quick Start

### Single Robot

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

### Multi-Robot Fleet (3 robots)

See [Multi-Robot Simulation](#multi-robot-simulation) section for full launch instructions.

### Send a navigation goal

```bash
ros2 topic pub --once /cpr_a200_0000/goal_waypoints geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "odom"}, pose: {position: {x: 8.0, y: 5.0, z: 0.0}, orientation: {w: 1.0}}}'
```

## Multi-Robot Simulation

Spawn 3 Huskies at different positions in the same Gazebo world:

```bash
# Terminal 1: Launch simulation with 3 robots
ros2 launch husky_bringup multi_sim.launch.py

# Terminals 2-4: Navigation stack for each robot
ros2 launch husky_bringup nav.launch.py namespace:=cpr_a200_0000
ros2 launch husky_bringup nav.launch.py namespace:=cpr_a200_0001
ros2 launch husky_bringup nav.launch.py namespace:=cpr_a200_0002

# Terminals 5-7: Mission executors
ros2 launch husky_bringup mission.launch.py namespace:=cpr_a200_0000 bt_file:=patrol_mission.xml
ros2 launch husky_bringup mission.launch.py namespace:=cpr_a200_0001 bt_file:=patrol_mission.xml
ros2 launch husky_bringup mission.launch.py namespace:=cpr_a200_0002 bt_file:=patrol_mission.xml

# Terminal 8: Fleet manager
ros2 launch husky_fleet_manager fleet_manager.launch.py
```

### Robot Configuration

| Namespace | Position | Config Directory |
|-----------|----------|------------------|
| `cpr_a200_0000` | x=0, y=0 | `/root/clearpath/` |
| `cpr_a200_0001` | x=3, y=0 | `/root/clearpath1/` |
| `cpr_a200_0002` | x=6, y=0 | `/root/clearpath2/` |

Each robot has its own `robot.yaml` with unique namespace and serial number. The fleet manager reads `/root/ros_ws/src/husky_fleet_manager/config/fleet.yaml` for home poses. Default world is `solar_farm`.

### Verification

```bash
# Check all robots are spawned
ros2 node list | grep mission_executor

# Check topics for each robot
ros2 topic list | grep cpr_a200

# Monitor fleet state
ros2 topic echo /fleet/robot_states
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
  "action": "navigate" | "set_state" | "rotate" | "emergency_stop" | "clear_emergency_stop" | "go_home" | "patrol",
  "robot_id": "cpr_a200_0000",
  "waypoints": [{"x": 5.0, "y": 0.0, "yaw": 1.57}],
  "waypoint_names": ["point_a", "point_b"],
  "relative": {"forward": 2.0, "right": 0.5},
  "priority": 3,
  "angle_deg": 180.0,
  "on_obstacle": "stop" | "return_home" | "skip_and_continue",
  "on_stuck": "abort" | "rotate_and_continue"
}]}
```

- `navigate`: Requires `waypoints[]` with x, y, optional yaw. Or `waypoint_names[]` for named locations. Or `relative` for body-frame moves.
- `rotate`: Requires `angle_deg` (-360 to 360). Positive = CCW.
- `set_state`: Sets `waiting` or `avoidance_enabled` flags.
- `emergency_stop`: Immediately stops the robot.
- `clear_emergency_stop`: Clears emergency stop flag.
- `go_home`: Sends robot to its home pose from fleet.yaml.
- `patrol`: Patrols multiple waypoints with contingency handling (`on_obstacle`, `on_stuck`).
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

### Fleet-level commands

The LLM can coordinate multiple robots in a single command:

```bash
# Send two robots to different locations
husky
> send robot 0 to point_a and robot 1 to point_b

# Keep one robot in reserve
husky
> send robot 0 to point_a, keep robot 1 in reserve

# All robots return home
husky
> all robots return to base
```

The LLM prompt includes all robot states, positions, and named waypoints. It automatically selects idle robots for parallel missions.

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

All nodes run under `cpr_a200_XXXX/` namespace. Multi-robot support uses different namespaces:

| Robot | Namespace | Config |
|-------|-----------|--------|
| Robot 0 | `cpr_a200_0000` | `/root/clearpath/robot.yaml` |
| Robot 1 | `cpr_a200_0001` | `/root/clearpath1/robot.yaml` |
| Robot 2 | `cpr_a200_0002` | `/root/clearpath2/robot.yaml` |

### Fleet Configuration

`/root/ros_ws/src/husky_fleet_manager/config/fleet.yaml` defines all robots and their home poses:

```yaml
fleet_manager:
  robots:
    - namespace: cpr_a200_0000
      home_pose: {x: 0.0, y: 0.0, yaw: 0.0}
    - namespace: cpr_a200_0001
      home_pose: {x: 3.0, y: 0.0, yaw: 0.0}
    - namespace: cpr_a200_0002
      home_pose: {x: 6.0, y: 0.0, yaw: 0.0}
```

The LLM bridge and validator also read this file to know which robot IDs are valid.

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

## Fleet Manager Interfaces

### Per-Robot (under namespace)

| Interface | Type | Path |
|-----------|------|------|
| `robot_state` | topic (RobotState) | `/cpr_a200_0000/robot_state` |
| `set_robot_state` | service (SetRobotState) | `/cpr_a200_0000/set_robot_state` |
| `navigate_to` | action (NavigateTo) | `/cpr_a200_0000/navigate_to` |

### Fleet-Level

| Interface | Type | Path |
|-----------|------|------|
| `fleet_navigate` | action (FleetNavigate) | `/fleet/fleet_navigate` |
| `set_fleet_state` | service (FleetSetState) | `/fleet/set_fleet_state` |
| `robot_states` | topic (FleetState) | `/fleet/robot_states` |

## Fleet Expansion Roadmap

The system is designed for multi-robot fleet coordination. Current status:

- ✅ **Phase 1: Multi-robot simulation** — 3 Huskies spawn in same Gazebo world
- 🔄 **Phase 2: Fleet manager activation** — Automatic goal reassignment on failure
- 🔄 **Phase 3: LLM fleet commands** — Fleet-level natural language coordination
- 📋 **Phase 4: Inter-robot collision avoidance** — Predictive path coordination
- 📋 **Phase 5: Outdoor testing** — Real hardware preparation

See `AGENTS.md` for detailed implementation plans.

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

### Multi-robot: robot not appearing in fleet state

Ensure all three robots have their mission executors running:
```bash
ros2 node list | grep mission_executor
```
Expected: `cpr_a200_0000/mission_executor`, `cpr_a200_0001/mission_executor`, `cpr_a200_0002/mission_executor`.

### Multi-robot: namespace collision

Each robot config directory must have a unique `namespace` in `robot.yaml`. Check:
```bash
grep namespace /root/clearpath*/robot.yaml
```

### Fleet manager: action server not available for multiple robots

The fleet manager waits 5s for each robot's action server. If a robot is slow to start, you may see warnings. Ensure all `mission.launch.py` instances are running before sending fleet commands.
