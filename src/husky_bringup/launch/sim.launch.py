# husky_bringup/launch/sim.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    return LaunchDescription([
        # Clearpath robot description (auto-generates URDF from YAML)
        IncludeLaunchDescription('clearpath_gz/launch/simulation.launch.py',
            launch_arguments={'robot_config': 'husky_a200.yaml',
                              'world': 'outdoor_empty.sdf'}.items()),
        # ros2_control controller manager is launched by clearpath_gz
        # diff_drive_controller spawner
        Node(package='controller_manager', executable='spawner',
             arguments=['diff_drive_controller', 'joint_state_broadcaster']),
    ])
