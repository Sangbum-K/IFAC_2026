from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='planning_pkg',
            executable='planning_node',
            name='planning',
            output='screen',
        ),
    ])
