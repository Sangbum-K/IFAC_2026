"""RViz 인터랙티브 레이스라인 튜너 (HMCL raceline_tuner의 ROS2 구현).

맵 + 튜너 노드 + RViz를 띄운다. 코너별로 라인/속도를 드래그로 조절:
  ros2 launch planning_pkg raceline_tuner.launch.py
  ros2 launch planning_pkg raceline_tuner.launch.py csv:=<다른 raceline>.csv

주의: 튜너는 '소스 트리'의 CSV를 직접 편집한다 (config/global_trajectory/).
저장 후 colcon build 해야 런타임 노드에 반영된다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 소스 트리 경로 (튜너는 install이 아니라 소스 CSV를 편집해야 재빌드로 반영됨)
SRC_CONFIG = os.path.expanduser(
    '~/IFAC_ws/planning_ws/src/planning_pkg/config/global_trajectory')


def launch_setup(context, *args, **kwargs):
    planning_share = get_package_share_directory('planning_pkg')
    csv_name = LaunchConfiguration('csv').perform(context)
    csv_path = os.path.join(SRC_CONFIG, csv_name)
    map_stem = os.path.join(planning_share, 'maps', 'Spielberg_map')

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[{'yaml_filename': map_stem + '.yaml'},
                    {'topic': 'map'},
                    {'frame_id': 'map'}],
    )
    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_tuner',
        parameters=[{'autostart': True},
                    {'node_names': ['map_server']}],
    )
    tuner_node = Node(
        package='planning_pkg',
        executable='raceline_tuner_node.py',
        name='raceline_tuner_node',
        parameters=[{'tuner/raceline_csv': csv_path}],
        output='screen',
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_tuner',
        arguments=['-d', os.path.join(planning_share, 'rviz', 'raceline_tuner.rviz')],
    )
    return [map_server_node, nav_lifecycle_node, tuner_node, rviz_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'csv',
            default_value='Spielberg_map_iqp_max7.00ms_margin0.15m_mu0.85.csv',
            description='편집할 raceline CSV 파일명 (config/global_trajectory/)'),
        OpaqueFunction(function=launch_setup),
    ])
