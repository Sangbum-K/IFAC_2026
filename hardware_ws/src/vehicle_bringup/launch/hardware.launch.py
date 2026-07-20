from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
import os


def _optional_external_launch(context, enabled_cfg, package_cfg, launch_file_cfg, label):
    enabled = enabled_cfg.perform(context).lower() == 'true'
    if not enabled:
        return []

    package_name = package_cfg.perform(context)
    launch_file = launch_file_cfg.perform(context)

    try:
        package_share = get_package_share_directory(package_name)
    except PackageNotFoundError:
        return [LogInfo(msg=f'[{label}] package "{package_name}" not found, skipping.')]

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(f'{package_share}/launch/{launch_file}')
        )
    ]


def generate_launch_description():
    env = LaunchConfiguration('env')
    enable_vesc = LaunchConfiguration('enable_vesc')
    enable_lidar = LaunchConfiguration('enable_lidar')
    enable_joy = LaunchConfiguration('enable_joy')
    enable_urdf = LaunchConfiguration('enable_urdf')
    enable_estop = LaunchConfiguration('enable_estop')
    vesc_package = LaunchConfiguration('vesc_package')
    vesc_launch = LaunchConfiguration('vesc_launch')
    lidar_package = LaunchConfiguration('lidar_package')
    lidar_launch = LaunchConfiguration('lidar_launch')
    vehicle_share = FindPackageShare('vehicle_bringup')

    base_config = PathJoinSubstitution([
        vehicle_share,
        'config',
        'params.yaml',
    ])
    env_config = PathJoinSubstitution([
        vehicle_share,
        'config',
        'environments',
        env,
    ])
    urdf_package_share = get_package_share_directory('vehicle_bringup')
    urdf_path = os.path.join(urdf_package_share, 'urdf', 'f1tenth_vehicle.urdf')

    with open(urdf_path, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    return LaunchDescription([
        DeclareLaunchArgument('env', default_value='real_car'),
        DeclareLaunchArgument('enable_vesc', default_value='true'),
        DeclareLaunchArgument('enable_lidar', default_value='true'),
        DeclareLaunchArgument('enable_joy', default_value='true'),
        DeclareLaunchArgument('enable_urdf', default_value='true'),
        DeclareLaunchArgument('enable_estop', default_value='true'),
        DeclareLaunchArgument('vesc_package', default_value='vesc_driver'),
        DeclareLaunchArgument('vesc_launch', default_value='vesc_driver_node.launch.py'),
        DeclareLaunchArgument('lidar_package', default_value='rplidar_ros'),
        DeclareLaunchArgument('lidar_launch', default_value='rplidar.launch.py'),
        OpaqueFunction(
            function=lambda context: _optional_external_launch(
                context, enable_vesc, vesc_package, vesc_launch, 'vesc'
            )
        ),
        OpaqueFunction(
            function=lambda context: _optional_external_launch(
                context, enable_lidar, lidar_package, lidar_launch, 'lidar'
            )
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                base_config,
                [env_config, '.yaml'],
                {'robot_description': robot_description},
            ],
            condition=IfCondition(enable_urdf),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_base_link_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[base_config, [env_config, '.yaml']],
            condition=IfCondition(enable_joy),
        ),
        Node(
            package='vehicle_bringup',
            executable='estop_node',
            name='estop_node',
            output='screen',
            parameters=[base_config, [env_config, '.yaml']],
            condition=IfCondition(enable_estop),
        ),
    ])
