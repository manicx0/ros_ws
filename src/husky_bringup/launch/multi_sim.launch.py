import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, PushLaunchConfigurations, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def set_resource_path(context, *args, **kwargs):
    ament_prefix_path = os.environ.get('AMENT_PREFIX_PATH', '')
    packages_share = [os.path.join(p, 'share') for p in ament_prefix_path.split(':') if p]

    pkg_clearpath_gz = get_package_share_directory('clearpath_gz')
    pkg_husky_bringup = get_package_share_directory('husky_bringup')

    paths = [
        os.path.join(pkg_husky_bringup, 'worlds'),
        os.path.join(pkg_clearpath_gz, 'worlds'),
        os.path.join(pkg_clearpath_gz, 'meshes'),
    ] + packages_share

    return [SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', ':'.join(paths))]


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_clearpath_gz = get_package_share_directory('clearpath_gz')

    gz_sim_launch = os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
    robot_spawn_launch = os.path.join(pkg_clearpath_gz, 'launch', 'robot_spawn.launch.py')

    set_gz_resource_path = OpaqueFunction(function=set_resource_path)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments=[
            ('gz_args', [LaunchConfiguration('world'), '.sdf', ' -r', ' -v 4',
                         PythonExpression(['\' -s\' if \'true\' == "',
                                           LaunchConfiguration('headless'), '" else \'\''])]),
        ])

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    robot0_spawn = GroupAction([
        PushLaunchConfigurations(),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_spawn_launch),
            launch_arguments=[
                ('use_sim_time', LaunchConfiguration('use_sim_time')),
                ('setup_path', '/root/clearpath/'),
                ('world', LaunchConfiguration('world')),
                ('rviz', 'false'),
                ('x', '0.0'),
                ('y', '0.0'),
                ('z', '0.3'),
                ('yaw', '0.0'),
                ('generate', 'false'),
            ])
    ])

    robot1_spawn = GroupAction([
        PushLaunchConfigurations(),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_spawn_launch),
            launch_arguments=[
                ('use_sim_time', LaunchConfiguration('use_sim_time')),
                ('setup_path', '/root/clearpath1/'),
                ('world', LaunchConfiguration('world')),
                ('rviz', 'false'),
                ('x', '3.0'),
                ('y', '0.0'),
                ('z', '0.3'),
                ('yaw', '0.0'),
                ('generate', 'false'),
            ])
    ])

    robot2_spawn = GroupAction([
        PushLaunchConfigurations(),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_spawn_launch),
            launch_arguments=[
                ('use_sim_time', LaunchConfiguration('use_sim_time')),
                ('setup_path', '/root/clearpath2/'),
                ('world', LaunchConfiguration('world')),
                ('rviz', 'false'),
                ('x', '6.0'),
                ('y', '0.0'),
                ('z', '0.3'),
                ('yaw', '0.0'),
                ('generate', 'false'),
            ])
    ])

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        choices=['true', 'false'],
        description='Use simulation time')
    world_arg = DeclareLaunchArgument(
        'world', default_value='solar_farm',
        description='Gazebo world')
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='true',
        choices=['true', 'false'],
        description='Run gz sim with -s (server only, no GUI) to save resources')

    ld = LaunchDescription([
        use_sim_time_arg,
        world_arg,
        headless_arg,
        set_gz_resource_path,
        gz_sim,
        clock_bridge,
        robot0_spawn,
        robot1_spawn,
        robot2_spawn,
    ])
    return ld
