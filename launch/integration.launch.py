"""전체 스택 진입점: integration_pkg stack.launch.py 위임.

  ros2 launch ~/IFAC_ws/launch/integration.launch.py map:=<맵이름>   # env:=sim gym 통합 시뮬
  ros2 launch ~/IFAC_ws/launch/integration.launch.py env:=real_car   # 실차 (hw/loc 별도)

env:=sim* 에서는 map 이 필수다 (기본 맵 없음 — stack.launch.py 가 목록을 띄우고 멈춘다).
원커맨드는 simm <맵이름>.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('env', default_value='sim',
                              description='sim | real_car | race_map | mapping'),
        # 기본 맵 없음: 맵 이름을 박아두면 옛 트랙 위에서 조용히 돈다.
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('integration_pkg'),
                'launch', 'stack.launch.py')),
            launch_arguments={
                'env': LaunchConfiguration('env'),
                'map': LaunchConfiguration('map'),
                'rviz': LaunchConfiguration('rviz'),
            }.items(),
        ),
    ])
