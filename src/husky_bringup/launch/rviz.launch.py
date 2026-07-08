from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


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

    pkg_husky_bringup = FindPackageShare('husky_bringup')
    rviz_config = PathJoinSubstitution(
        [pkg_husky_bringup, 'rviz', 'nav.rviz'])

    rviz_group = GroupAction([
        PushRosNamespace(namespace),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ],
            output='screen'),
    ])

    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,
        rviz_group,
    ])
