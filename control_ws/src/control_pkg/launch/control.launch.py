from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    env = LaunchConfiguration('env')
    control_share = FindPackageShare('control_pkg')
    base_config = PathJoinSubstitution([control_share, 'config', 'params.yaml'])
    env_config_dir = PathJoinSubstitution([control_share, 'config', 'environments'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'env',
            default_value='sim',
            description='Configuration profile: sim, mapping, race_map, real_car',
        ),
        Node(
            package='control_pkg',
            executable='control_node',
            name='control',
            output='screen',
            parameters=[
                base_config,
                [env_config_dir, '/', env, '.yaml'],
            ],
        ),
    ])
