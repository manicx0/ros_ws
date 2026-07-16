import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_husky_bringup = get_package_share_directory('husky_bringup')

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='cpr_a200_0000',
        description='Robot namespace'
    )

    navsat_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[os.path.join(pkg_husky_bringup, 'config', 'navsat_params.yaml')],
        remappings=[
            ('imu/data', 'imu/data'),
            ('gps/fix', 'sensors/gps_0/fix'),
            ('odometry/filtered', 'platform/odom/filtered'),
            ('gps/filtered', 'gps/filtered'),
            ('odometry/gps', 'odometry/gps'),
        ]
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_husky_bringup, 'config', 'gps_ekf_params.yaml')],
        remappings=[
            ('odometry/filtered', 'platform/odom/filtered'),
        ]
    )

    return LaunchDescription([
        namespace_arg,
        PushRosNamespace(LaunchConfiguration('namespace')),
        navsat_node,
        ekf_node,
    ])
