from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    env = LaunchConfiguration('env')
    package_share = FindPackageShare('perception_pkg')
    base_config = PathJoinSubstitution([package_share, 'config', 'params.yaml'])
    env_config = PathJoinSubstitution([package_share, 'config', 'environments', env])

    return LaunchDescription([
        DeclareLaunchArgument('env', default_value='sim'),
        Node(
            package='perception_pkg',
            executable='obstacle_detector_node',
            name='obstacle_detector_node',
            output='screen',
            parameters=[base_config, [env_config, '.yaml']],
        ),
    ])
