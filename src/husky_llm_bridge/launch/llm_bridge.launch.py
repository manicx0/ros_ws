import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    connector_arg = DeclareLaunchArgument(
        'connector',
        default_value='gemini_connector_node.py',
        description='Which LLM connector to launch: gemini_connector_node.py or ollama_connector_node.py'
    )

    include_fleet_manager_arg = DeclareLaunchArgument(
        'include_fleet_manager',
        default_value='true',
        description='Whether to include the fleet manager node'
    )

    fleet_config_path = os.path.join(
        get_package_share_directory('husky_fleet_manager'),
        'config',
        'fleet.yaml'
    )

    waypoints_config_path = os.path.join(
        get_package_share_directory('husky_llm_bridge'),
        'config',
        'waypoints.yaml'
    )

    bridge_node = Node(
        package='husky_llm_bridge',
        executable='llm_bridge_node.py',
        name='llm_bridge',
        output='screen',
        parameters=[{'fleet_config_path': fleet_config_path, 'waypoints_config_path': waypoints_config_path}]
    )

    validator_node = Node(
        package='husky_llm_bridge',
        executable='llm_validator_node.py',
        name='llm_validator',
        output='screen',
        parameters=[{'fleet_config_path': fleet_config_path}]
    )

    connector_node = Node(
        package='husky_llm_bridge',
        executable=LaunchConfiguration('connector'),
        name='llm_connector',
        output='screen',
        parameters=[{'fleet_config_path': fleet_config_path, 'waypoints_config_path': waypoints_config_path}]
    )

    fleet_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('husky_fleet_manager'),
                'launch',
                'fleet_manager.launch.py'
            )
        ),
        condition=IfCondition(LaunchConfiguration('include_fleet_manager'))
    )

    return LaunchDescription([
        connector_arg,
        include_fleet_manager_arg,
        fleet_manager_launch,
        bridge_node,
        validator_node,
        connector_node,
    ])
