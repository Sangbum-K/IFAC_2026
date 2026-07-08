from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    env = LaunchConfiguration('env')
    sensor_interface = LaunchConfiguration('sensor_interface')
    enable_teleop = LaunchConfiguration('enable_teleop')
    speed_to_erpm_gain = LaunchConfiguration('speed_to_erpm_gain')
    speed_to_erpm_offset = LaunchConfiguration('speed_to_erpm_offset')
    steering_angle_to_servo_gain = LaunchConfiguration('steering_angle_to_servo_gain')
    steering_angle_to_servo_offset = LaunchConfiguration('steering_angle_to_servo_offset')
    wheelbase = LaunchConfiguration('wheelbase')
    publish_tf = LaunchConfiguration('publish_tf')
    backend = LaunchConfiguration('backend')
    mode = LaunchConfiguration('mode')

    return LaunchDescription([
        DeclareLaunchArgument('env', default_value='mapping'),
        DeclareLaunchArgument('sensor_interface', default_value='ethernet'),
        DeclareLaunchArgument('enable_teleop', default_value='true'),
        DeclareLaunchArgument('speed_to_erpm_gain', default_value='4614.0'),
        DeclareLaunchArgument('speed_to_erpm_offset', default_value='0.0'),
        DeclareLaunchArgument('steering_angle_to_servo_gain', default_value='-1.2135'),
        DeclareLaunchArgument('steering_angle_to_servo_offset', default_value='0.5304'),
        DeclareLaunchArgument('wheelbase', default_value='0.33'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('backend', default_value='slam_toolbox'),
        DeclareLaunchArgument('mode', default_value='mapping'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('f1tenth_stack'), 'launch', 'bringup.launch.py'])
            ),
            launch_arguments={
                'env': env,
                'sensor_interface': sensor_interface,
                'enable_teleop': enable_teleop,
                'speed_to_erpm_gain': speed_to_erpm_gain,
                'speed_to_erpm_offset': speed_to_erpm_offset,
                'steering_angle_to_servo_gain': steering_angle_to_servo_gain,
                'steering_angle_to_servo_offset': steering_angle_to_servo_offset,
                'wheelbase': wheelbase,
                'publish_tf': publish_tf,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('localization_bringup'),
                    'launch',
                    'localization.launch.py',
                ])
            ),
            launch_arguments={
                'backend': backend,
                'mode': mode,
            }.items(),
        ),
    ])
