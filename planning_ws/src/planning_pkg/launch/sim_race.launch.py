"""RViz 시뮬레이션 데모 launch (unicorn 튜토리얼의 race.launch sim:=true 대응).

f1tenth_gym_ros 시뮬레이터(2-agent) + planning 스택 + pure pursuit 컨트롤러 +
Pitwall 시뮬 노드들 + RViz를 한 번에 실행한다.

  ros2 launch planning_pkg sim_race.launch.py                    # Spielberg (기본)
  ros2 launch planning_pkg sim_race.launch.py map:=Budapest_map  # 헝가로링 (코너 다수)
  ros2 launch planning_pkg sim_race.launch.py map:=my_track      # 실차 트랙 (suffix 자동)
  ros2 launch planning_pkg sim_race.launch.py rviz:=false

맵 추가 방법: maps/에 <맵>_map.{png,yaml}을 넣고
scripts/global_traj_sim/step4_main.py --map <맵>_map --raceline iqp \
    --safe-dist 0.5 --v-max 15 로 raceline을 생성한 뒤 colcon build.
시작 포즈는 raceline CSV 첫 행에서 자동으로 읽고, 상대 주차 위치만
아래 PARK_POSES에 추가하면 된다 (레이스라인에서 30m 이상 떨어진 맵 구석).

상대 차량은 실제 gym 2번째 에이전트(자체 LiDAR/물리 포함)로, 시작 시
맵 구석 '주차장'에 있다 (ego LiDAR 사거리 밖 = 없는 것과 동일).

RViz 사용법 (unicorn 튜토리얼과 동일):
  - "Publish Point"로 트랙 위 클릭 -> 정적 장애물 생성 -> spliner가 회피
    (같은 장애물 근처를 다시 클릭하면 제거, 패널 버튼으로 전체 삭제)
  - "2D Goal Pose" -> 상대 차량을 그 위치로 텔레포트 = 스폰
  - "2D Pose Estimate" -> ego 위치 재배치 (gym_bridge 지원)
  - 초록 반투명 라인 = 추월 섹터 (/planner/ot_sectors/markers)
  - Pitwall 패널 (Panels > PitwallPanel): STATE 배너 / 이벤트 로그 /
    ego Autonomous·Manual / 상대 모드·속도±·제거 / 장애물 Clear /
    LiDAR 토글 / Perception 주입 모드

전제: ~/sim_ws 에 f1tenth_gym_ros가 빌드돼 있어야 한다.
  source ~/sim_ws/install/setup.bash 후 planning_ws setup.bash를 source.
"""

import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

# 상대 차량 '주차장' (제거 시 이동 위치): 레이스라인에서 최대한 떨어진 맵 구석.
# 맵에 없으면 아래 launch_setup이 맵 경계 밖 구석으로 자동 폴백한다.
PARK_POSES = {
    'Spielberg_map': (-79.81, -31.26),
    'Budapest_map': (-56.0, 103.0),
    'my_map': (-20.0, -20.0),   # 트랙(x:18~25, y:-3~8)에서 ~47m 떨어진 구석
    # my_track은 19.5x10.5m로 작아 LiDAR(≈30m) 밖에 못 숨긴다. 레이스라인
    # (x:[-7.8,5.0] y:[-0.4,5.1]) 바깥 좌상단 구석 — 외벽에 가려지길 기대.
    # 상대가 유령 장애물로 보이면 정적 장애물(Publish Point)로 회피를 시험할 것.
    'my_track': (-12.0, 8.5),
}


def _read_start_pose(csv_path):
    """raceline CSV 첫 행에서 ego 시작 포즈 (x, y, psi)를 읽는다."""
    with open(csv_path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            cols = [float(c) for c in line.split(';')]
            return cols[1], cols[2], cols[5]
    raise RuntimeError(f'raceline CSV가 비어 있음: {csv_path}')


def launch_setup(context, *args, **kwargs):
    planning_share = get_package_share_directory('planning_pkg')
    gym_share = get_package_share_directory('f1tenth_gym_ros')

    params_file = os.path.join(planning_share, 'config', 'params.yaml')
    gym_config = os.path.join(gym_share, 'config', 'sim.yaml')

    map_name = LaunchConfiguration('map').perform(context)
    map_stem = os.path.join(planning_share, 'maps', map_name)
    if not os.path.exists(map_stem + '.yaml'):
        raise RuntimeError(f'맵 없음: {map_stem}.yaml (maps/ 에 추가 필요)')

    gt_dir = os.path.join(planning_share, 'config', 'global_trajectory')
    suffix = LaunchConfiguration('suffix').perform(context)
    if not suffix:
        # suffix 미지정 시 해당 맵의 iqp CSV를 찾아 접미사를 자동 추론한다.
        # 맵마다 raceline 파라미터가 달라 하나의 고정 접미사로는 안 맞기 때문
        # (Spielberg/Budapest = max15..., my_track/my_map = max6...).
        iqp_glob = sorted(glob.glob(os.path.join(gt_dir, f'{map_name}_iqp_*.csv')))
        if not iqp_glob:
            raise RuntimeError(
                f'raceline 없음: {gt_dir}/{map_name}_iqp_*.csv '
                '(step4_main.py로 생성 후 colcon build 필요)')
        traj_csv = os.path.basename(iqp_glob[0])
        suffix = traj_csv[len(f'{map_name}_iqp_'):-len('.csv')]
    else:
        traj_csv = f'{map_name}_iqp_{suffix}.csv'
    ot_csv = f'{map_name}_sp_{suffix}.csv'
    center_csv = f'{map_name}_centerline.csv'
    traj_csv_path = os.path.join(gt_dir, traj_csv)
    sx, sy, stheta = _read_start_pose(traj_csv_path)
    # 맵이 PARK_POSES에 없으면 시작점에서 멀찍이 떨어진 곳으로 폴백(KeyError 방지)
    park_x, park_y = PARK_POSES.get(map_name, (sx - 40.0, sy - 40.0))

    # gym 기본 설정 위에 맵과 시작 포즈를 덮어쓴다.
    # 상대(agent 2)는 주차장(레이스라인에서 30m+, ego LiDAR 밖)에서 시작 —
    # RViz "2D Goal Pose"로 트랙에 텔레포트하면 스폰된다.
    gym_overrides = {
        'map_path': map_stem,
        'map_img_ext': '.png',
        'num_agent': 2,
        'sx': sx,
        'sy': sy,
        'stheta': stheta,
        'sx1': park_x,
        'sy1': park_y,
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

    # perception:=lidar — 실제 perception 파이프라인 통합 모드:
    #   가상 장애물은 /scan_overlay 합성점으로만 노출(GT 주입 중단)되고,
    #   perception_pkg obstacle_detector(/scan_overlay -> /perception/obstacles_detected)
    #   -> perception_obstacle_bridge(-> /perception/obstacles_frenet) -> spliner.
    #   detector/control은 integration_pkg stack.launch.py가 별도로 띄운다.
    perception_mode = LaunchConfiguration('perception').perform(context)
    lidar_perception = (perception_mode == 'lidar')

    stack_args = {
        'odom_mode': 'sim_gt',
        'trajectory_csv_file': traj_csv,
        'overtaking_csv_file': ot_csv,
        'centerline_csv_file': center_csv,
        # recovery yaw 소스: sim은 pf가 없으므로 gym GT odom 사용
        'pose_odom_topic': '/ego_racecar/odom',
    }
    if lidar_perception:
        stack_args.update({
            'obstacle_topic': '/perception/obstacles_frenet',
            'bridge': 'true',
            'bridge_input_topic': '/perception/obstacles_detected',
        })

    planner_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(planning_share, 'launch', 'planner_stack.launch.py')),
        launch_arguments=stack_args.items(),
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
        parameters=[params_file,
                    {'virtual_obstacle/perception_mode':
                        'lidar_overlay' if lidar_perception else 'tracking_merge'}],
        output='screen',
    )
    # 상대 차량 드라이버 (gym agent 2: Path/FTG/Manual) + Predictive Spliner 예측기
    opponent_driver_node = Node(
        package='planning_pkg',
        executable='opponent_driver_node.py',
        name='opponent_driver_node',
        parameters=[params_file,
                    {'opponent/park_x': park_x, 'opponent/park_y': park_y}],
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
            + traj_csv)}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('tuner')),
    )
    # lidar 모드에선 raw frenet 관측이 /perception/obstacles_frenet으로 오므로 remap
    opponent_predictor_node = Node(
        package='planning_pkg',
        executable='opponent_predictor_node.py',
        name='opponent_predictor_node',
        parameters=[params_file],
        remappings=([('/perception/obstacles', '/perception/obstacles_frenet')]
                    if lidar_perception else []),
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(planning_share, 'rviz', 'sim_race.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return [
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
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='Spielberg_map',
                              description='맵 이름 (maps/<이름>.yaml, ' +
                                          ', '.join(PARK_POSES)),
        DeclareLaunchArgument('suffix', default_value='',
                              description='raceline CSV 접미사 (iqp/sp 공통). '
                                          '비워두면 맵의 iqp CSV에서 자동 추론 '
                                          '(예: my_track=max6.00ms_margin0.15m_mu0.75). '
                                          '한 맵에 여러 raceline이 있을 때만 지정.'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='RViz 실행 여부'),
        DeclareLaunchArgument('tracker', default_value='true',
                              description='Python 트래커 실행 여부 (false면 C++ control_node 사용)'),
        DeclareLaunchArgument('tuner', default_value='false',
                              description='레이스라인 튜너 함께 실행 (같은 RViz에서 편집)'),
        DeclareLaunchArgument('perception', default_value='ground_truth',
                              description='ground_truth: 가상 장애물 GT 직접 주입(기존), '
                                          'lidar: 실제 perception 체인 '
                                          '(/scan_overlay -> detector -> bridge -> spliner)'),
        OpaqueFunction(function=launch_setup),
    ])
