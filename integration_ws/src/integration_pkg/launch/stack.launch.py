"""전체 스택 통합 launch (perception + planning + control + integration 모니터).

  # gym 통합 시뮬 (f1tenth_gym_ros + 실제 perception 체인 + C++ control_node):
  # env:=sim* 에서는 map:=<맵이름>이 필수다 (기본 맵 없음).
  ros2 launch integration_pkg stack.launch.py map:=<맵이름>
  ros2 launch integration_pkg stack.launch.py map:=<맵이름> env:=sim_real_car  # 실차 근사
  ros2 launch integration_pkg stack.launch.py map:=<맵이름> rviz:=false
  # 원커맨드(맵 준비+빌드까지): simm <맵이름>

  # 실차 (hardware/localization은 각 팀 launch로 별도 기동):
  ros2 launch integration_pkg stack.launch.py env:=real_car

env:=sim 데이터 흐름 (실차와 동일 토폴로지를 gym 위에 재현):
  gym /scan -> virtual_obstacle(lidar_overlay, 클릭 장애물 합성) -> /scan_overlay
    -> perception_pkg obstacle_detector -> /perception/obstacles_detected (laser frame)
    -> planning perception_obstacle_bridge -> /perception/obstacles_frenet
    -> spliner/recovery.  control_ws control_node가 /local_waypoints -> /drive.

env:=real_car 전제:
  perception 팀 노드가 /perception/obstacles(f1tenth_perception_msgs) 발행,
  localization_ws particle_filter가 /pf/pose/odom + map->odom TF 발행,
  hardware_ws가 /scan·/odom·estop 제공. 여기서는 perception/planning/control/
  integration만 기동한다.

전제 source 순서: ros humble -> (sim이면 ~/sim_ws) -> planning_ws -> perception_ws
  -> control_ws -> integration_ws.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    env = LaunchConfiguration('env').perform(context)
    planning_share = get_package_share_directory('planning_pkg')
    perception_share = get_package_share_directory('perception_pkg')
    control_share = get_package_share_directory('control_pkg')
    integration_share = get_package_share_directory('integration_pkg')

    actions = []

    map_name = LaunchConfiguration('map').perform(context).strip()
    trajectory_csv = LaunchConfiguration('trajectory_csv_file').perform(context).strip()
    suffix = LaunchConfiguration('suffix').perform(context).strip()

    if env in ('sim', 'sim_real_car'):
        if not map_name:
            raise RuntimeError(
                f'env:={env} 에는 map:=<맵이름> 이 필요합니다 (기본 맵 없음).\n'
                '  맵 이름을 여기에 박아두면 실차 트랙이 바뀔 때마다 launch를 고쳐야 하고,\n'
                '  안 고치면 옛 트랙 위에서 조용히 돈다 — 그래서 호출자가 준다.\n'
                '  원커맨드: simm <맵이름>  (CSV 생성 + 빌드 + 실행)\n'
                '  맵 목록: ls ~/IFAC_ws/maps/*.yaml')
        # gym + planner 스택 + sim 보조 노드 + RViz.
        # tracker:=false — python tracker 대신 control_ws C++ control_node 사용.
        # perception:=lidar — GT 주입 대신 실제 perception 체인.
        #
        # env:=sim_real_car 는 "최대한 실차와 같게" 프로파일이다:
        #   - perception/planning/control 이 environments/sim_real_car.yaml 사용
        #     (거동 파라미터는 real_car.yaml과 동일, 배선만 sim)
        #   - pf:=true      : GT가 아니라 진짜 particle_filter로 위치추정
        #   - vehicle:=real_car : gym 차량 물리를 실측값(mu 0.90 / a_max 4.5 /
        #     조향 좌 0.3800·우 -0.3578 / v_max 7.0)으로 — 고속 슬립이 실차와 같은
        #     지점에서 나게 한다. LiDAR 40 Hz·라이다 x 0.28 m 도 여기서 온다
        #     (단일 기준은 sim_race.launch.py REAL_CAR_VEHICLE. 여기 적힌
        #      mu 0.75 / a_max 3.5 / s_max 0.36 은 08-10 동기화 이전 값이었다)
        #   - map 기본값도 실차가 지금 달리는 트랙
        real_car_like = (env == 'sim_real_car')
        pf_arg = LaunchConfiguration('pf').perform(context).strip().lower()
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(planning_share, 'launch', 'sim_race.launch.py')),
            launch_arguments={
                # 맵은 호출자가 준다. sim / sim_real_car 의 차이는 맵이 아니라
                # 프로파일이다:
                #   sim          = gym 기본 물리 + real_car 파라미터 + GT odom
                #   sim_real_car = 실측 물리 + real_car 파라미터 + PF
                # (2026-08-12부터 둘의 **파라미터는 같다**. 차이는 위치추정과
                #  차량물리뿐이라, sim 은 "빠른 확인", sim_real_car 는 "실차 근사"다.)
                #
                # ⚠️ 어떤 맵이든 "그 맵의 raceline이 지금 배포된 종방향 상수로
                # 생성됐는지"는 별개다 (V_MAX / A_ACC / A_DEC_PLANNED / MU_PROFILE,
                # <맵>_generation_manifest.json 참고). 옛 맵을 쓰면 gym 차량 물리
                # (vehicle:=real_car)만 새 상수고 궤적은 옛 상수라 어긋난다 —
                # `simm <맵> --regen` 으로 다시 뽑으면 항상 현재 상수로 맞는다.
                'map': map_name,
                # 맵마다 iqp CSV가 여러 벌이면 launch의 glob이 "임의 선택하지
                # 않음"으로 죽는다 (예: my_map은 mu0.65/mu0.75 두 벌 ->
                # suffix:=max6.00ms_margin0.08m_mu0.65 필요). 한 벌이면 자동 추론.
                'suffix': suffix,
                'rviz': LaunchConfiguration('rviz').perform(context),
                'tracker': 'false',
                'perception': 'lidar',
                # planner_stack이 environments/<env>.yaml을 덮어쓰게 한다.
                # env:=sim 도 2026-08-12부터 sim.yaml 이 real_car 거동을 덮어쓴다
                # (예전엔 비어 있어서 params.yaml 저속 기본값 그대로였다).
                'env': env,
                # pf:=true — GT 대신 particle_filter. bridge/use_map_filter 처럼
                # "임계값이 PF 오차보다 큰가"를 묻는 로직은 GT에선 무조건
                # 통과하므로, 그걸 검증하려면 이쪽을 켜야 한다.
                # sim_real_car는 실차가 PF를 쓰므로 기본 true.
                'pf': pf_arg or ('true' if real_car_like else 'false'),
                'vehicle': 'real_car' if real_car_like else 'gym_default',
                'map_filter': LaunchConfiguration('map_filter').perform(context),
                'maps_dir': LaunchConfiguration('maps_dir').perform(context),
            }.items(),
        ))
    else:
        # 실차/race_map: map:=<이름>이 주어졌을 때만 CSV 3종을 그 맵으로 바꾼다.
        # 빈 값이면 params.yaml 값을 그대로 둔다 (기존 동작).
        planner_args = {'env': env}
        if map_name:
            planner_args['map'] = map_name
        if trajectory_csv:
            planner_args['trajectory_csv_file'] = trajectory_csv
        if suffix:
            planner_args['suffix'] = suffix
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(planning_share, 'launch', 'planner_stack.launch.py')),
            launch_arguments=planner_args.items(),
        ))

    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perception_share, 'launch', 'perception.launch.py')),
        launch_arguments={'env': env}.items(),
    ))
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(control_share, 'launch', 'control.launch.py')),
        launch_arguments={'env': env}.items(),
    ))
    actions.append(Node(
        package='integration_pkg',
        executable='integration_node',
        name='integration',
        output='screen',
        parameters=[
            os.path.join(integration_share, 'config', 'params.yaml'),
            os.path.join(integration_share, 'config', 'environments', env + '.yaml'),
        ],
    ))
    return actions



def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('env', default_value='sim',
                              description='sim | sim_real_car | real_car | '
                                          'race_map | mapping. sim_real_car = gym '
                                          '위에서 실차 파라미터+실차 차량물리+PF'),
        DeclareLaunchArgument('map', default_value='',
                              description='맵 이름. env:=sim* 이면 **필수** '
                                          '(기본 맵 없음). 그 외에는 planner의 '
                                          'CSV 3종을 이 맵으로 전환하고, 빈 값이면 '
                                          'params.yaml 값 유지. ⚠️ raceline CSV가 '
                                          '있는 맵만 쓸 수 있다 '
                                          '(config/global_trajectory/<맵>_iqp_*.csv)'),
        DeclareLaunchArgument(
            'trajectory_csv_file', default_value='',
            description='(env:=real_car|race_map) 사용할 정확한 raceline CSV '
                        '파일명 (config/global_trajectory/)'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='(env:=sim) RViz 실행 여부'),
        DeclareLaunchArgument('maps_dir', default_value='',
                              description='(env:=sim*) 맵 폴더. 빈 값이면 '
                                          '워크스페이스 공용 ~/IFAC_ws/maps'),
        DeclareLaunchArgument('suffix', default_value='',
                              description='raceline CSV 접미사. '
                                          '빈 값이면 맵의 iqp CSV에서 자동 추론 '
                                          '(여러 개면 launch가 거부하므로 명시 필요)'),
        DeclareLaunchArgument('map_filter', default_value='',
                              description='(env:=sim) bridge/use_map_filter override '
                                          '(true|false). 벽 클러스터 제거 A/B 확인용'),
        DeclareLaunchArgument('pf', default_value='',
                              description='(env:=sim*) true면 gym GT 대신 '
                                          'particle_filter로 위치추정 — 실차와 같은 '
                                          '위치추정 오차 위에서 인지를 검증할 때 사용. '
                                          '빈 값이면 env:=sim_real_car는 true, '
                                          'env:=sim은 false가 기본.'),
        OpaqueFunction(function=launch_setup),
    ])
