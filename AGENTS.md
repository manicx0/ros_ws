# Husky A200 workspace — agent cheat sheet

## Build

```bash
cd /root/ros_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install
```

## Robot config (critical — must parse without errors)

Config at `/root/clearpath/robot.yaml`. The A200 URDF macro defines `top_chassis_link`, not `top_plate_link` — sensor `parent:` must match. After editing, validate with:

```bash
source install/setup.bash
ros2 run clearpath_generator_common generate_description -s /root/clearpath/
ros2 run clearpath_generator_common generate_semantic_description -s /root/clearpath/  # catches missing links
ros2 run clearpath_generator_gz generate_launch -s /root/clearpath/
ros2 run clearpath_generator_gz generate_param -s /root/clearpath/
```

If `generate_semantic_description` fails on "parent link not found", the sensor parent in `robot.yaml` is wrong.

## Launch order (5 terminals)

| Terminal | Command |
|----------|---------|
| 1 | `ros2 launch husky_bringup sim.launch.py` |
| 2 | `ros2 launch husky_bringup nav.launch.py namespace:=cpr_a200_0000` |
| 3 | `ros2 launch husky_bringup rviz.launch.py namespace:=cpr_a200_0000` |
| 4 | `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=True -r /cmd_vel:=/cpr_a200_0000/cmd_vel` |
| 5 | `ros2 launch husky_bringup mission.launch.py namespace:=cpr_a200_0000 bt_file:=patrol_mission.xml` |

All commands need `source install/setup.bash` first. Teleop requires a real TTY — fails in headless/script contexts.

## Architecture

- `src/husky_nav/` — pure_pursuit, path_planner, obstacle_detector, ekf_gps (all C++ nodes). Topic strings use relative paths (no leading `/`) to inherit namespace from `PushRosNamespace`.
- `src/husky_bt/` — BehaviorTree.CPP mission executor, BT node plugins (navigate_to_goal, obstacle_check, recovery_rotate). XML trees in `bt_xml/`.
- `src/husky_bringup/` — launch files, configs, worlds, RViz config.
- `src/husky_msgs/` — custom messages (RobotState, GoalEvent, FleetState, FleetGoal, FleetResult), services (SetRobotState, FleetSetState), actions (NavigateTo, FleetNavigate).
- `src/husky_fleet_manager/` — C++ node: aggregates per-robot states into `/fleet/robot_states`, dispatches goals via `/fleet/fleet_navigate` action, batch state control via `/fleet/set_fleet_state` service.
- `src/husky_llm_bridge/` — Python skeleton: subscribes fleet state, provides `send_fleet_goals()` and `set_fleet_state()` methods. No LLM API integration yet.

## Known limitations (V1)

- **NavigateTo action feedback uses `string status`** — the `.action` file defines
  `string status` where the fleet manager would write
  `if (feedback->status == "navig")`. This is fragile and typo-prone. V2 should
  change to `uint8 status` with action-internal constants
  (`NAVIGATING=1, OBSTACLE_BLOCKED=2, GOAL_REACHED=3, GOAL_FAILED=4`) so the
  fleet manager writes
  `if (feedback->status == NavigateTo::Feedback::OBSTACLE_BLOCKED)` — compile-
  time safe, no string parsing. The `GoalEvent` topic stays as strings (fine for
  monitoring/broadcast).

## Controllers

`gz_ros_control` inside Gazebo auto-loads `joint_state_broadcaster` and `platform_velocity_controller` from the spawned robot URDF. The `spawner` nodes in `control.launch.py` retry on failure for 60s. Normal output:

```
[spawner-*] Configured and activated joint_state_broadcaster
[spawner-*] Configured and activated platform_velocity_controller
```

## Key topics (under `cpr_a200_0000/` namespace)

| Topic | Type |
|-------|------|
| `cmd_vel` | `TwistStamped` — input from teleop/nav |
| `platform/cmd_vel` | diff_drive controller input |
| `platform/joint_states` | wheel joint states |
| `platform/odom` | diff_drive odometry |
| `velodyne_points` | `PointCloud2` — full 3D lidar |
| `scan_2d` | `LaserScan` — 2D projection |
| `goal_waypoints` | `PoseStamped` — autonomous goal |
| `global_path` | `Path` — planned path |

## Navigation goal

```bash
ros2 topic pub --once /cpr_a200_0000/goal_waypoints geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "odom"}, pose: {position: {x: 8.0, y: 5.0, z: 0.0}, orientation: {w: 1.0}}}'
```

## Repo layout

```
src/
  clearpath_common/      — upstream (gitignored, system-installed at /opt/ros/jazzy)
  clearpath_gz/           — upstream (gitignored, system-installed at /opt/ros/jazzy)
  husky_bringup/          — custom launch + config
  husky_bt/               — behavior tree nodes + executor (state-aware BT)
  husky_nav/              — navigation stack nodes
  husky_msgs/             — custom messages, services, and actions
  husky_fleet_manager/    — fleet-level state aggregation and goal dispatch
  husky_llm_bridge/       — LLM bridge skeleton (Python)
```

## Session summary (Jul 2026 — BT states + fleet manager)

### Packages built/modified

| Package | Role |
|---------|------|
| `husky_msgs` | Custom messages: RobotState, GoalEvent, FleetState, FleetGoal, FleetResult; services: SetRobotState, FleetSetState; actions: NavigateTo, FleetNavigate |
| `husky_bt` | BehaviorTree.CPP executor with state-aware nodes |
| `husky_fleet_manager` | C++ node: aggregates robot states, dispatches goals across fleet |
| `husky_llm_bridge` | Python skeleton: subscribes fleet state, provides send_goals/set_state methods (no LLM API yet) |

### State ownership matrix

| Flag | Owned by | Set by | Cleared by |
|------|----------|--------|------------|
| `emergency_stop` | Topic subscriber (read-only to LLM) | `emergency_stop` topic (hardware) | `set_robot_state` service |
| `waiting` | LLM (operator) | `set_robot_state` service | `set_robot_state` service |
| `idle` | BT (IdleMonitor) | BT when goal queue empty | BT when new goal arrives |
| `avoidance_enabled` | LLM (operator) | `set_robot_state` service | `set_robot_state` service |
| `mission_active` | BT (NavigateToGoal) | BT on mission start | BT on mission end |
| `has_goal` | BT (internal) | NavigateTo action server | NavigateToGoal on success |

### Robot interfaces (per-namespace)

| Interface | Type | Example path |
|-----------|------|-------------|
| `robot_state` | topic (RobotState) | `/cpr_a200_0000/robot_state` |
| `set_robot_state` | service (SetRobotState) | `/cpr_a200_0000/set_robot_state` |
| `navigate_to` | action (NavigateTo) | `/cpr_a200_0000/navigate_to` |
| `goal_events` | topic (GoalEvent) | `/fleet/goal_events` (global) |
| `emergency_stop` | topic (Bool) | `/cpr_a200_0000/emergency_stop` |

### Fleet interfaces

| Interface | Type | Path |
|-----------|------|------|
| `fleet_navigate` | action (FleetNavigate) | `/fleet/fleet_navigate` |
| `set_fleet_state` | service (FleetSetState) | `/fleet/set_fleet_state` |
| `robot_states` | topic (FleetState) | `/fleet/robot_states` |

### Testing (single robot)

```bash
# Check state
ros2 topic echo /cpr_a200_0000/robot_state
# Set waiting
ros2 service call /cpr_a200_0000/set_robot_state husky_msgs/srv/SetRobotState '{waiting: true, avoidance_enabled: true}'
# Send navigation goal
ros2 action send_goal /cpr_a200_0000/navigate_to husky_msgs/action/NavigateTo \
  '{target_pose: {header: {frame_id: odom}, pose: {position: {x: 8.0, y: 5.0, z: 0.0}, orientation: {w: 1.0}}}}' --feedback
# Emergency stop
ros2 topic pub --once /cpr_a200_0000/emergency_stop std_msgs/msg/Bool '{data: true}'
```

### 4 commits in this sprint

```
d1f6f82 feat: implement BT state architecture for fleet LLM control
a6d8a23 feat: add NavigateTo action server for goal assignment
1132b14 feat: add fleet manager and LLM bridge for multi-robot coordination
757c07d fix: resolve blocking calls, config loading, and cmd_vel type issues
```

### Known rough edges

- LLM bridge has no LLM API input (no REST/WebSocket/stdin) — skeleton only
- Fleet manager uses raw BT.CPP (not nav2_behavior_tree) — matches existing codebase
- No multi-robot simulation infrastructure to test fleet manager end-to-end
- Navsat/GPS plugin not installed yet

## Gotchas

- `sim.launch.py` uses `ros_gz_sim/launch/gz_sim.launch.py` directly (not clearpath_gz's wrapper). It sets `GZ_SIM_RESOURCE_PATH` via an `OpaqueFunction` before Gazebo starts.
- Default world is `warehouse` (shelves, chairs, people — real obstacles).
- `enable_odom_tf: False` in control.yaml — EKF publishes odom→base_link instead.
- The `clearpath_common` and `clearpath_gz` submodules under `src/` are stubs/shadowed; the real packages live at `/opt/ros/jazzy/`.