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
    ])
