"""RViz 시뮬레이션 데모 launch (unicorn 튜토리얼의 race.launch sim:=true 대응).

f1tenth_gym_ros 시뮬레이터(Spielberg 맵) + planning 스택 + pure pursuit
컨트롤러 + 가상 장애물 노드 + RViz를 한 번에 실행한다.

  ros2 launch planning_pkg sim_race.launch.py
  ros2 launch planning_pkg sim_race.launch.py rviz:=false

RViz 사용법 (unicorn 튜토리얼과 동일):
  - "Publish Point"로 트랙 위 클릭 -> 정적 장애물 생성 -> spliner가 회피
  - 같은 장애물 근처를 다시 클릭 -> 장애물 제거
  - "2D Pose Estimate" -> 차량 위치 재배치 (gym_bridge 지원)

전제: ~/sim_ws 에 f1tenth_gym_ros가 빌드돼 있어야 한다.
  source ~/sim_ws/install/setup.bash 후 planning_ws setup.bash를 source.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    planning_share = get_package_share_directory('planning_pkg')
    gym_share = get_package_share_directory('f1tenth_gym_ros')

    params_file = os.path.join(planning_share, 'config', 'params.yaml')
    gym_config = os.path.join(gym_share, 'config', 'sim.yaml')
    map_stem = os.path.join(planning_share, 'maps', 'Spielberg_map')

    # gym 기본 설정 위에 Spielberg 맵과 raceline 시작 포즈만 덮어쓴다
    gym_overrides = {
        'map_path': map_stem,
        'map_img_ext': '.png',
        'num_agent': 1,
        'sx': 0.15,       # raceline 시작점 (config/global_trajectory CSV 첫 행)
        'sy': -0.688,
        'stheta': 0.262,
    }

    bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        parameters=[gym_config, gym_overrides],
    )
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[{'yaml_filename': map_stem + '.yaml'},
                    {'topic': 'map'},
                    {'frame_id': 'map'},
                    {'use_sim_time': True}],
    )
    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}],
    )
    ego_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ego_robot_state_publisher',
        parameters=[{'robot_description': Command(
            ['xacro ', os.path.join(gym_share, 'launch', 'ego_racecar.xacro')])}],
        remappings=[('/robot_description', 'ego_robot_description')],
    )

    planner_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(planning_share, 'launch', 'planner_stack.launch.py')),
        launch_arguments={'odom_mode': 'sim_gt'}.items(),
    )

    # tracker:=false 로 끄면 control_ws의 C++ control_node로 대체 가능:
    #   ros2 run control_pkg control_node --ros-args \
    #     --params-file <control_pkg>/config/params.yaml \
    #     --params-file <control_pkg>/config/environments/sim.yaml
    tracker_node = Node(
        package='planning_pkg',
        executable='waypoint_tracker_node.py',
        name='waypoint_tracker_node',
        parameters=[params_file],
        output='screen',
        condition=IfCondition(LaunchConfiguration('tracker')),
    )
    virtual_obstacle_node = Node(
        package='planning_pkg',
        executable='virtual_obstacle_node.py',
        name='virtual_obstacle_node',
        parameters=[params_file],
        output='screen',
    )
    # 가상 동적 상대 (2D Goal Pose로 스폰) + Predictive Spliner 예측기
    virtual_opponent_node = Node(
        package='planning_pkg',
        executable='virtual_opponent_node.py',
        name='virtual_opponent_node',
        parameters=[params_file],
        output='screen',
    )
    opponent_predictor_node = Node(
        package='planning_pkg',
        executable='opponent_predictor_node.py',
        name='opponent_predictor_node',
        parameters=[params_file],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(planning_share, 'rviz', 'sim_race.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='RViz 실행 여부'),
        DeclareLaunchArgument('tracker', default_value='true',
                              description='Python 트래커 실행 여부 (false면 C++ control_node 사용)'),
        bridge_node,
        map_server_node,
        nav_lifecycle_node,
        ego_robot_publisher,
        planner_stack,
        tracker_node,
        virtual_obstacle_node,
        virtual_opponent_node,
        opponent_predictor_node,
        rviz_node,
    ])
