Updated Testing Workflow
# Terminal 1: Gazebo + Husky + warehouse obstacles
source install/setup.bash
ros2 launch husky_bringup sim. launch.py
# Wait for robot to appear in Gazebo (chassis, 4 wheels, lidar all visible now)
# Terminal 2: Navigation stack
source install/setup.bash
ros2 launch husky_bringup nav.launch.py namespace:=cpr_a200_0000
# Terminal 3: RViz with custom nav display
source install/setup.bash
ros2 launch husky_bringup rviz.launch.py namespace:=cpr_a200_0000
_0000
# Terminal 4: Manual teleop (test robot drives)
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard |
--ros-args -r /cmd vel:=/cpr a200 0000/cmd vel
# Terminal 5: Send an autonomous goal
source install/setup.bash ros2 topic pub /cpr_a200
a200 0000/goal waypoints geometry msgs/PoseStamped
'{header: {frame_id: "odom"}, pose: {position: {x: 8.0, y: 5.0, z: 0.0}, orientation: {w: 1.0}
}}'
- - once
