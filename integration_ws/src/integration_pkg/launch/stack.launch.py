from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    env_arg = DeclareLaunchArgument(
        'env',
        default_value='sim',
        description='Configuration profile: sim, mapping, race_map, real_car',
    )

    env = LaunchConfiguration('env')

    perception_base_config = os.path.join(
        get_package_share_directory('perception_pkg'),
        'config',
        'params.yaml',
    )
    perception_env_config = os.path.join(
        get_package_share_directory('perception_pkg'),
        'config',
        'environments',
    )
    planning_base_config = os.path.join(
        get_package_share_directory('planning_pkg'),
        'config',
        'params.yaml',
    )
    planning_env_config = os.path.join(
        get_package_share_directory('planning_pkg'),
        'config',
        'environments',
    )
    control_base_config = os.path.join(
        get_package_share_directory('control_pkg'),
        'config',
        'params.yaml',
    )
    control_env_config = os.path.join(
        get_package_share_directory('control_pkg'),
        'config',
        'environments',
    )
    integration_base_config = os.path.join(
        get_package_share_directory('integration_pkg'),
        'config',
        'params.yaml',
    )
    integration_env_config = os.path.join(
        get_package_share_directory('integration_pkg'),
        'config',
        'environments',
    )
    return LaunchDescription([
        env_arg,
        Node(
            package='perception_pkg',
            executable='perception_node',
            name='perception',
            output='screen',
            parameters=[
                perception_base_config,
                [perception_env_config, '/', env, '.yaml'],
            ],
        ),
        Node(
            package='planning_pkg',
            executable='planning_node',
            name='planning',
            output='screen',
            parameters=[
                planning_base_config,
                [planning_env_config, '/', env, '.yaml'],
            ],
        ),
        Node(
            package='control_pkg',
            executable='control_node',
            name='control',
            output='screen',
            parameters=[
                control_base_config,
                [control_env_config, '/', env, '.yaml'],
            ],
        ),
        Node(
            package='integration_pkg',
            executable='integration_node',
            name='integration',
            output='screen',
            parameters=[
                integration_base_config,
                [integration_env_config, '/', env, '.yaml'],
            ],
        ),
    ])
