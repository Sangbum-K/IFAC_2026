# MIT License

# Copyright (c) 2025 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    joy_teleop_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'joy_teleop.yaml'
    )
    vesc_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'vesc.yaml'
    )
    # Read the current calibration values straight out of vesc.yaml so it stays
    # the single source of truth: editing vesc.yaml alone changes the defaults
    # below, while a `:=` launch argument can still override for quick tuning.
    with open(vesc_config, 'r') as f:
        vesc_defaults = yaml.safe_load(f)['/**']['ros__parameters']
    sensors_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'sensors.yaml'
    )
    mux_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'mux.yaml'
    )

    joy_la = DeclareLaunchArgument(
        'joy_config',
        default_value=joy_teleop_config,
        description='Descriptions for joy and joy_teleop configs')
    vesc_la = DeclareLaunchArgument(
        'vesc_config',
        default_value=vesc_config,
        description='Descriptions for vesc configs')
    sensors_la = DeclareLaunchArgument(
        'sensors_config',
        default_value=sensors_config,
        description='Descriptions for sensor configs')
    mux_la = DeclareLaunchArgument(
        'mux_config',
        default_value=mux_config,
        description='Descriptions for ackermann mux configs')

    # VESC calibration overrides - exposed as launch args so they can be
    # tuned from the command line without editing vesc.yaml each time.
    speed_to_erpm_gain_la = DeclareLaunchArgument(
        'speed_to_erpm_gain',
        default_value=str(vesc_defaults['speed_to_erpm_gain']),
        description='erpm = speed_to_erpm_gain * speed (m/s) + speed_to_erpm_offset')
    speed_to_erpm_offset_la = DeclareLaunchArgument(
        'speed_to_erpm_offset',
        default_value=str(vesc_defaults['speed_to_erpm_offset']),
        description='erpm = speed_to_erpm_gain * speed (m/s) + speed_to_erpm_offset')
    steering_angle_to_servo_gain_la = DeclareLaunchArgument(
        'steering_angle_to_servo_gain',
        default_value=str(vesc_defaults['steering_angle_to_servo_gain']),
        description='servo = steering_angle_to_servo_gain * steering angle (rad) + steering_angle_to_servo_offset')
    steering_angle_to_servo_offset_la = DeclareLaunchArgument(
        'steering_angle_to_servo_offset',
        default_value=str(vesc_defaults['steering_angle_to_servo_offset']),
        description='servo = steering_angle_to_servo_gain * steering angle (rad) + steering_angle_to_servo_offset')

    vesc_calibration_params = {
        'speed_to_erpm_gain': LaunchConfiguration('speed_to_erpm_gain'),
        'speed_to_erpm_offset': LaunchConfiguration('speed_to_erpm_offset'),
        'steering_angle_to_servo_gain': LaunchConfiguration('steering_angle_to_servo_gain'),
        'steering_angle_to_servo_offset': LaunchConfiguration('steering_angle_to_servo_offset'),
    }

    ld = LaunchDescription([
        joy_la, vesc_la, sensors_la, mux_la,
        speed_to_erpm_gain_la, speed_to_erpm_offset_la,
        steering_angle_to_servo_gain_la, steering_angle_to_servo_offset_la,
    ])

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        parameters=[LaunchConfiguration('joy_config')]
    )
    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[LaunchConfiguration('joy_config')]
    )
    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        parameters=[LaunchConfiguration('vesc_config'), vesc_calibration_params]
    )
    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        parameters=[LaunchConfiguration('vesc_config'), vesc_calibration_params]
    )
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    throttle_interpolator_node = Node(
        package='f1tenth_stack',
        executable='throttle_interpolator',
        name='throttle_interpolator',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    urg_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        parameters=[LaunchConfiguration('sensors_config')]
    )
    ackermann_mux_node = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        parameters=[LaunchConfiguration('mux_config')],
        remappings=[('ackermann_drive_out', 'ackermann_cmd')]
    )
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
        arguments=['0.27', '0.0', '0.11', '0.0', '0.0', '0.0', 'base_link', 'laser']
    )

    # finalize
    ld.add_action(joy_node)
    ld.add_action(joy_teleop_node)
    ld.add_action(ackermann_to_vesc_node)
    ld.add_action(vesc_to_odom_node)
    ld.add_action(vesc_driver_node)
    # ld.add_action(throttle_interpolator_node)
    ld.add_action(urg_node)
    ld.add_action(ackermann_mux_node)
    ld.add_action(static_tf_node)

    return ld