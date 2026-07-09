import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('husky_fleet_manager'),
        'config',
        'fleet.yaml'
    )

    return LaunchDescription([
        Node(
            package='husky_fleet_manager',
            executable='fleet_manager_node',
            name='fleet_manager',
            parameters=[{'fleet_config': config_file}],
            output='screen'
        )
    ])
