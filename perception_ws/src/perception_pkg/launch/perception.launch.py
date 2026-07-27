"""LiDAR 장애물 인지 + 시각화.

  ros2 launch perception_pkg perception.launch.py env:=real_car
  ros2 launch perception_pkg perception.launch.py env:=real_car viz:=false

viz 노드는 인지 결과를 /perception/obstacle_markers로 그려서 monitor.rviz의
"Obstacles" 디스플레이에 띄운다. 인지 체인에 영향을 주지 않는 순수 구독자라
기본 on — 회피 라인이 왜 그렇게 휘었는지 대조하려면 필요하다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
        DeclareLaunchArgument(
            'viz', default_value='true',
            description='장애물 마커(/perception/obstacle_markers) 발행 여부'),
        Node(
            package='perception_pkg',
            executable='obstacle_detector_node',
            name='obstacle_detector_node',
            output='screen',
            parameters=[base_config, [env_config, '.yaml']],
        ),
        Node(
            package='perception_pkg',
            executable='obstacle_viz_node',
            name='obstacle_viz_node',
            output='screen',
            parameters=[base_config, [env_config, '.yaml']],
            condition=IfCondition(LaunchConfiguration('viz')),
        ),
    ])
