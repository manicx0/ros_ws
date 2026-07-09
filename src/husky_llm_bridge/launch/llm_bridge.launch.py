import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='husky_llm_bridge',
            executable='llm_bridge_node.py',
            name='llm_bridge',
            output='screen'
        )
    ])
