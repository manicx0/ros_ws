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
| 4 | `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cpr_a200_0000/cmd_vel` |
| 5 | `ros2 launch husky_bringup mission.launch.py namespace:=cpr_a200_0000 bt_file:=patrol_mission.xml` |

All commands need `source install/setup.bash` first. Teleop requires a real TTY — fails in headless/script contexts.

## Architecture

- `src/husky_nav/` — pure_pursuit, path_planner, obstacle_detector, ekf_gps (all C++ nodes). Topic strings use relative paths (no leading `/`) to inherit namespace from `PushRosNamespace`.
- `src/husky_bt/` — BehaviorTree.CPP mission executor, BT node plugins (navigate_to_goal, obstacle_check, recovery_rotate). XML trees in `bt_xml/`.
- `src/husky_bringup/` — launch files, configs, worlds, RViz config.

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
  clearpath_common/   — upstream (gitignored, system-installed at /opt/ros/jazzy)
  clearpath_gz/        — upstream (gitignored, system-installed at /opt/ros/jazzy)
  husky_bringup/       — custom launch + config
  husky_bt/            — behavior tree nodes + executor
  husky_nav/           — navigation stack nodes
```

## Gotchas

- `sim.launch.py` uses `ros_gz_sim/launch/gz_sim.launch.py` directly (not clearpath_gz's wrapper). It sets `GZ_SIM_RESOURCE_PATH` via an `OpaqueFunction` before Gazebo starts.
- Default world is `warehouse` (shelves, chairs, people — real obstacles).
- `enable_odom_tf: False` in control.yaml — EKF publishes odom→base_link instead.
- The `clearpath_common` and `clearpath_gz` submodules under `src/` are stubs/shadowed; the real packages live at `/opt/ros/jazzy/`.