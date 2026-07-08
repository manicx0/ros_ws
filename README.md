# Husky A200 Autonomous Navigation (ROS 2 Jazzy + Gazebo Harmonic)

Custom navigation stack (no Nav2) for Clearpath Husky A200 with VLP-16 lidar, using pure pursuit, obstacle detection, path planning, and BehaviorTree.CPP mission execution.

## Quick Start

### Prerequisites

The robot configuration lives at `/root/clearpath/robot.yaml`. It must have:

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
      ros_parameters:
        velodyne_driver_node:
          model: VLP16
          frame_id: lidar3d_0_laser
          device_ip: 192.168.131.25
          port: 2368
        velodyne_transform_node:
          model: VLP16
          fixed_frame: lidar3d_0_laser
          target_frame: lidar3d_0_laser
```

### Terminal 1: Gazebo + Robot

```bash
source install/setup.bash
ros2 launch husky_bringup sim.launch.py
```

Wait for the robot to appear in the Gazebo entity tree (chassis, 4 wheels, lidar visible). Controllers will self-configure within ~30s.

### Terminal 2: Navigation Stack

```bash
source install/setup.bash
ros2 launch husky_bringup nav.launch.py namespace:=cpr_a200_0000
```

Starts pure pursuit controller, path planner, obstacle detector, and EKF.

### Terminal 3: RViz Visualization

```bash
source install/setup.bash
ros2 launch husky_bringup rviz.launch.py namespace:=cpr_a200_0000
```

Displays robot model, TF, point cloud, planned path, and odometry.

### Terminal 4: Manual Teleop

```bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/cpr_a200_0000/cmd_vel
```

Use keyboard to drive the robot (i/j/k/l keys).

### Terminal 5: Send Autonomous Goal

```bash
source install/setup.bash
ros2 topic pub --once /cpr_a200_0000/goal_waypoints geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "odom"}, pose: {position: {x: 8.0, y: 5.0, z: 0.0}, orientation: {w: 1.0}}}'
```

### Terminal 6: Mission Execution (BehaviorTree)

```bash
source install/setup.bash
ros2 launch husky_bringup mission.launch.py namespace:=cpr_a200_0000 bt_file:=patrol_mission.xml
```

## Troubleshooting

### Robot not visible in Gazebo

Check the generation pipeline:

```bash
source install/setup.bash
ros2 run clearpath_generator_common generate_description -s /root/clearpath/
ros2 run clearpath_generator_common generate_semantic_description -s /root/clearpath/
ros2 run clearpath_generator_gz generate_launch -s /root/clearpath/
ros2 run clearpath_generator_gz generate_param -s /root/clearpath/
```

If `generate_semantic_description` fails with "parent link not found", check `robot.yaml` sensor `parent:` field — A200 URDF uses `top_chassis_link`, not `top_plate_link`.

### Controllers fail to configure

The spawners have a 60s timeout. If they still fail, check:

```bash
ros2 control list_controllers
ros2 control list_hardware_components
```

Ensure the URDF uses `gz_ros2_control/GazeboSimSystem` (for simulation) not `clearpath_hardware_interfaces/A200Hardware`. Run `xacro robot.urdf.xacro is_sim:=true` to verify.

### Teleop crashes

```
termios.error: (25, 'Inappropriate ioctl for device')
```

This is a non-TTY environment issue. Run `teleop_twist_keyboard` in a **real terminal window**, not through a script or headless session.
