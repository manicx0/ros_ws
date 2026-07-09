from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    husky_bringup_dir = get_package_share_directory('husky_bringup')

    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Robot namespace')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        choices=['true', 'false'],
        description='Use simulation time')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    nav_group = GroupAction([
        PushRosNamespace(namespace),

        Node(
            package='husky_nav',
            executable='obstacle_detector_node',
            name='obstacle_detector',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        Node(
            package='husky_nav',
            executable='pure_pursuit_node',
            name='pure_pursuit',
            output='screen',
            parameters=[
                os.path.join(husky_bringup_dir, 'config', 'pure_pursuit_params.yaml'),
                {'use_sim_time': use_sim_time},
            ],
        ),

        Node(
            package='husky_nav',
            executable='path_planner_node',
            name='path_planner',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        Node(
            package='husky_nav',
            executable='goal_pose_relay_node',
            name='goal_pose_relay',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])

    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,
        nav_group,
    ])
