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

    vehicle_bringup_share = FindPackageShare('vehicle_bringup')
    urg_share = FindPackageShare('urg_node')

    env_config = PathJoinSubstitution([stack_share, 'config', 'environments', env])
    joy_config = PathJoinSubstitution([stack_share, 'config', 'joy_teleop.yaml'])
    mux_config = PathJoinSubstitution([stack_share, 'config', 'mux.yaml'])
    vesc_config = PathJoinSubstitution([stack_share, 'config', 'vesc.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('env', default_value='real_car'),
        DeclareLaunchArgument('sensor_interface', default_value='ethernet'),
        DeclareLaunchArgument('enable_teleop', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([vehicle_bringup_share, 'launch', 'hardware.launch.py'])
            ),
            launch_arguments={
                'env': env,
                'enable_lidar': 'false',
                'enable_joy': 'false',
                'enable_vesc': 'true',
                'enable_urdf': 'true',
                'enable_estop': 'true',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([urg_share, 'launch', 'urg_node_launch.py'])
            ),
            launch_arguments={'sensor_interface': sensor_interface}.items(),
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy',
            output='screen',
            parameters=[joy_config, [env_config, '.yaml']],
        ),
        Node(
            package='joy_teleop',
            executable='joy_teleop',
            name='joy_teleop',
            output='screen',
            parameters=[joy_config, [env_config, '.yaml']],
            condition=IfCondition(enable_teleop),
        ),
        Node(
            package='ackermann_mux',
            executable='ackermann_mux',
            name='ackermann_mux',
            output='screen',
            parameters=[mux_config, [env_config, '.yaml']],
            remappings=[('ackermann_drive_out', 'ackermann_cmd')],
        ),
        Node(
            package='f1tenth_stack',
            executable='autonomy_hold_gate',
            name='autonomy_hold_gate',
            output='screen',
            parameters=[{
                'joy_topic': '/joy',
                'input_drive_topic': '/drive',
                'output_drive_topic': '/drive_hold',
                'enable_button': 5,
                'command_timeout': 0.3,
                'publish_rate': 30.0,
            }],
        ),
        Node(
            package='vesc_ackermann',
            executable='ackermann_to_vesc_node',
            name='ackermann_to_vesc_node',
            output='screen',
            parameters=[vesc_config, [env_config, '.yaml']],
        ),
        Node(
            package='vesc_ackermann',
            executable='vesc_to_odom_node',
            name='vesc_to_odom_node',
            output='screen',
            parameters=[vesc_config, [env_config, '.yaml']],
        ),
    ])
