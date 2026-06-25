from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os


def generate_launch_description():
    here = os.path.dirname(__file__)
    ld = LaunchDescription()

    # Include perception
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(here, 'perception.launch.py'))
    ))

    # Include planning
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(here, 'planning.launch.py'))
    ))

    # Include control
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(here, 'control.launch.py'))
    ))

    # Add a simple bridge/status node
    ld.add_action(Node(
        package='rclpy',
        executable='lifecycle_manager',
        name='integration_status',
        output='screen',
    ))

    return ld
