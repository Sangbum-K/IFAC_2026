"""주행 상태 모니터 + RViz.

  ros2 launch monitor_pkg monitor.launch.py                 # sim (RViz 포함)
  ros2 launch monitor_pkg monitor.launch.py env:=real_car   # 실차
  ros2 launch monitor_pkg monitor.launch.py rviz:=false     # 노드만 (RViz는 노트북에서 별도 실행)

실차에서는 차량 NUC에서 rviz:=false로 노드만 띄우고, 같은 ROS_DOMAIN_ID의
노트북에서 `rviz2 -d <planning_ws>/src/monitor_pkg/rviz/monitor.rviz`를 여는 구성 권장.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix, get_package_share_directory


def launch_setup(context, *args, **kwargs):
    env = LaunchConfiguration('env')

    monitor_share = get_package_share_directory('monitor_pkg')
    monitor_prefix = get_package_prefix('monitor_pkg')
    base_config = os.path.join(monitor_share, 'config', 'params.yaml')
    env_config = os.path.join(monitor_share, 'config', 'environments')
    monitor_executable = os.path.join(monitor_prefix, 'lib', 'monitor_pkg', 'monitor_node')

    actions = []
    if os.path.exists(monitor_executable):
        actions.append(Node(
            package='monitor_pkg',
            executable='monitor_node',
            name='monitor_node',
            output='screen',
            parameters=[
                base_config,
                [env_config, '/', env, '.yaml'],
            ],
        ))
    else:
        actions.append(LogInfo(
            msg='[monitor_pkg] monitor_node was not built. Install '
                'ros-humble-rviz-2d-overlay-msgs and '
                'ros-humble-rviz-2d-overlay-plugins, then rebuild planning_ws '
                'to enable the RViz overlay monitor.'
        ))

    actions.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_monitor',
        arguments=['-d', os.path.join(monitor_share, 'rviz', 'monitor.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('env', default_value='sim',
                              description='Configuration profile: sim, real_car, race_map'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz with monitor overlay config'),
        OpaqueFunction(function=launch_setup),
    ])
