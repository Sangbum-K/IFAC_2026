import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    env = LaunchConfiguration('env')
    sensor_interface = LaunchConfiguration('sensor_interface')
    enable_teleop = LaunchConfiguration('enable_teleop')
    enable_estop = LaunchConfiguration('enable_estop')
    speed_to_erpm_gain = LaunchConfiguration('speed_to_erpm_gain')
    speed_to_erpm_offset = LaunchConfiguration('speed_to_erpm_offset')
    steering_angle_to_servo_gain = LaunchConfiguration('steering_angle_to_servo_gain')
    steering_angle_to_servo_offset = LaunchConfiguration('steering_angle_to_servo_offset')
    wheelbase = LaunchConfiguration('wheelbase')
    publish_tf = LaunchConfiguration('publish_tf')

    vehicle_bringup_share = FindPackageShare('vehicle_bringup')
    urg_share = FindPackageShare('urg_node')
    vesc_share = FindPackageShare('vesc_driver')
    urdf_package_share = get_package_share_directory('vehicle_bringup')
    urdf_path = os.path.join(urdf_package_share, 'urdf', 'f1tenth_vehicle.urdf')

    base_config = PathJoinSubstitution([vehicle_bringup_share, 'config', 'params.yaml'])
    env_config = PathJoinSubstitution([vehicle_bringup_share, 'config', 'environments', env])

    with open(urdf_path, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    return LaunchDescription([
        DeclareLaunchArgument('env', default_value='real_car'),
        DeclareLaunchArgument('sensor_interface', default_value='ethernet'),
        DeclareLaunchArgument('enable_teleop', default_value='true'),
        DeclareLaunchArgument('enable_estop', default_value='true'),
        DeclareLaunchArgument('speed_to_erpm_gain', default_value='4614.0'),
        DeclareLaunchArgument('speed_to_erpm_offset', default_value='0.0'),
        DeclareLaunchArgument('steering_angle_to_servo_gain', default_value='-1.2135'),
        DeclareLaunchArgument('steering_angle_to_servo_offset', default_value='0.5304'),
        DeclareLaunchArgument('wheelbase', default_value='0.33'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([vesc_share, 'launch', 'vesc_driver_node.launch.py'])
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([urg_share, 'launch', 'urg_node_launch.py'])
            ),
            launch_arguments={'sensor_interface': sensor_interface}.items(),
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
        ),
        Node(
            package='vehicle_bringup',
            executable='estop_node',
            name='estop_node',
            output='screen',
            parameters=[base_config, [env_config, '.yaml']],
            condition=IfCondition(enable_estop),
        ),
        Node(
            package='vehicle_teleop',
            executable='ackermann_joy_teleop',
            name='ackermann_joy_teleop',
            output='screen',
            parameters=[base_config, [env_config, '.yaml']],
            condition=IfCondition(enable_teleop),
        ),
        Node(
            package='vehicle_teleop',
            executable='ackermann_cmd_mux',
            name='ackermann_cmd_mux',
            output='screen',
            parameters=[base_config, [env_config, '.yaml']],
        ),
        Node(
            package='vesc_ackermann',
            executable='ackermann_to_vesc_node',
            name='ackermann_to_vesc_node',
            output='screen',
            parameters=[{
                'speed_to_erpm_gain': speed_to_erpm_gain,
                'speed_to_erpm_offset': speed_to_erpm_offset,
                'steering_angle_to_servo_gain': steering_angle_to_servo_gain,
                'steering_angle_to_servo_offset': steering_angle_to_servo_offset,
            }],
        ),
        Node(
            package='vesc_ackermann',
            executable='vesc_to_odom_node',
            name='vesc_to_odom_node',
            output='screen',
            parameters=[{
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'speed_to_erpm_gain': speed_to_erpm_gain,
                'speed_to_erpm_offset': speed_to_erpm_offset,
                'use_servo_cmd_to_calc_angular_velocity': True,
                'steering_angle_to_servo_gain': steering_angle_to_servo_gain,
                'steering_angle_to_servo_offset': steering_angle_to_servo_offset,
                'wheelbase': wheelbase,
                'publish_tf': publish_tf,
            }],
        ),
    ])
