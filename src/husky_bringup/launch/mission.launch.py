from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Robot namespace')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        choices=['true', 'false'],
        description='Use simulation time')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    bt_group = GroupAction([
        PushRosNamespace(namespace),

        Node(
            package='husky_bt',
            executable='mission_executor_node',
            name='bt_mission_executive',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])

    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,
        bt_group,
    ])
