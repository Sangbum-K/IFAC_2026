"""RViz 시뮬레이션 데모 launch (unicorn 튜토리얼의 race.launch sim:=true 대응).

f1tenth_gym_ros 시뮬레이터(Spielberg 맵, 2-agent) + planning 스택 +
pure pursuit 컨트롤러 + Pitwall 시뮬 노드들 + RViz를 한 번에 실행한다.

  ros2 launch planning_pkg sim_race.launch.py
  ros2 launch planning_pkg sim_race.launch.py rviz:=false

상대 차량은 실제 gym 2번째 에이전트(자체 LiDAR/물리 포함)로, 시작 시
맵 구석 '주차장'에 있다 (ego LiDAR 사거리 밖 = 없는 것과 동일).

RViz 사용법 (unicorn 튜토리얼과 동일):
  - "Publish Point"로 트랙 위 클릭 -> 정적 장애물 생성 -> spliner가 회피
    (같은 장애물 근처를 다시 클릭하면 제거, 패널 버튼으로 전체 삭제)
  - "2D Goal Pose" -> 상대 차량을 그 위치로 텔레포트 = 스폰
  - "2D Pose Estimate" -> ego 위치 재배치 (gym_bridge 지원)
  - Pitwall 패널 (Panels > PitwallPanel): STATE 배너 / 이벤트 로그 /
    ego Autonomous·Manual / 상대 모드·속도±·제거 / 장애물 Clear /
    LiDAR 토글 / Perception 주입 모드

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

    # gym 기본 설정 위에 Spielberg 맵과 시작 포즈를 덮어쓴다.
    # 상대(agent 2)는 주차장(레이스라인에서 ~49m, ego LiDAR 밖)에서 시작 —
    # RViz "2D Goal Pose"로 트랙에 텔레포트하면 스폰된다.
    gym_overrides = {
        'map_path': map_stem,
        'map_img_ext': '.png',
        'num_agent': 2,
        'sx': 0.15,       # raceline 시작점 (config/global_trajectory CSV 첫 행)
        'sy': -0.688,
        'stheta': 0.262,
        'sx1': -79.81,    # 주차장 (opponent_driver_node opponent/park_* 와 일치)
        'sy1': -31.26,
        'stheta1': 0.0,
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
    opp_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='opp_robot_state_publisher',
        parameters=[{'robot_description': Command(
            ['xacro ', os.path.join(gym_share, 'launch', 'opp_racecar.xacro')])}],
        remappings=[('/robot_description', 'opp_robot_description')],
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
    # 상대 차량 드라이버 (gym agent 2: Path/FTG/Manual) + Predictive Spliner 예측기
    opponent_driver_node = Node(
        package='planning_pkg',
        executable='opponent_driver_node.py',
        name='opponent_driver_node',
        parameters=[params_file],
        output='screen',
    )
    # 레이스라인 튜너 (tuner:=true 로 시뮬과 함께 실행 — 같은 RViz에서
    # 보라 핸들 드래그로 라인/속도 편집. 소스 CSV를 편집하므로 저장 후 재빌드)
    tuner_node = Node(
        package='planning_pkg',
        executable='raceline_tuner_node.py',
        name='raceline_tuner_node',
        parameters=[{'tuner/raceline_csv': os.path.expanduser(
            '~/IFAC_ws/planning_ws/src/planning_pkg/config/global_trajectory/'
            'Spielberg_map_iqp_max7.00ms_margin0.15m_mu0.85.csv')}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('tuner')),
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
        DeclareLaunchArgument('tuner', default_value='false',
                              description='레이스라인 튜너 함께 실행 (같은 RViz에서 편집)'),
        bridge_node,
        map_server_node,
        nav_lifecycle_node,
        ego_robot_publisher,
        opp_robot_publisher,
        planner_stack,
        tracker_node,
        tuner_node,
        virtual_obstacle_node,
        opponent_driver_node,
        opponent_predictor_node,
        rviz_node,
    ])
