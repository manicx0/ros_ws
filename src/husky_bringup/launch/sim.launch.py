import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
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
            ('gz_args', [LaunchConfiguration('world'), '.sdf', ' -r', ' -v 4']),
        ])

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_spawn_launch),
        launch_arguments=[
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('setup_path', LaunchConfiguration('setup_path')),
            ('world', LaunchConfiguration('world')),
            ('rviz', LaunchConfiguration('rviz')),
            ('x', LaunchConfiguration('x')),
            ('y', LaunchConfiguration('y')),
            ('z', LaunchConfiguration('z')),
            ('yaw', LaunchConfiguration('yaw')),
            ('generate', 'false'),
        ])

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        choices=['true', 'false'],
        description='Use simulation time')
    world_arg = DeclareLaunchArgument(
        'world', default_value='warehouse',
        description='Gazebo world')
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        choices=['true', 'false'],
        description='Start RViz')
    setup_path_arg = DeclareLaunchArgument(
        'setup_path', default_value='/root/clearpath/',
        description='Clearpath setup path')

    pose_args = []
    for element in ['x', 'y', 'yaw']:
        pose_args.append(DeclareLaunchArgument(
            element, default_value='0.0',
            description=f'{element} component of the robot pose'))
    pose_args.append(DeclareLaunchArgument(
        'z', default_value='0.3',
        description='z component of the robot pose'))

    ld = LaunchDescription([
        use_sim_time_arg,
        world_arg,
        rviz_arg,
        setup_path_arg,
        *pose_args,
        set_gz_resource_path,
        gz_sim,
        clock_bridge,
        robot_spawn,
    ])
    return ld
