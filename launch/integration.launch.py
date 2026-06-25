from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='perception_pkg',
            executable='perception_node',
            name='perception',
            output='screen',
        ),
        Node(
            package='planning_pkg',
            executable='planning_node',
            name='planning',
            output='screen',
        ),
        Node(
            package='control_pkg',
            executable='control_node',
            name='control',
            output='screen',
        ),
        Node(
            package='integration_pkg',
            executable='integration_node',
            name='integration',
            output='screen',
        ),
    ])
