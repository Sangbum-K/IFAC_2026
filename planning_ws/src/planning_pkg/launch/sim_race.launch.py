"""RViz 시뮬레이션 데모 launch (unicorn 튜토리얼의 race.launch sim:=true 대응).

f1tenth_gym_ros 시뮬레이터(2-agent) + planning 스택 + pure pursuit 컨트롤러 +
Pitwall 시뮬 노드들 + RViz를 한 번에 실행한다.

  ros2 launch planning_pkg sim_race.launch.py map:=<맵이름>     # map은 필수
  ros2 launch planning_pkg sim_race.launch.py map:=<맵이름> rviz:=false

**기본 맵은 없다.** 예전엔 '지금 실차가 달리는 트랙' 이름이 기본값으로 박혀
있었는데, 맵이 바뀔 때마다 이 파일을 고쳐야 했고 안 고치면 옛 트랙 위에서
조용히 도는 사고가 났다. 이제 맵 이름은 호출자가 준다 (원커맨드: `simm <맵>`).

⚠️ 쓸 수 있는 맵은 "<ws>/maps 에 있고 raceline CSV 도 있는" 맵뿐이다
   (config/global_trajectory/<맵>_iqp_*.csv). 맵만 있고 CSV 가 없으면
   launch 가 사용 가능한 맵 목록을 띄우고 죽는다 — build_map.py 로 먼저 생성할 것.

맵 추가 방법: 워크스페이스 공용 맵 폴더 **<ws>/maps/** 에 <맵>.{png,pgm,yaml}을
넣고 scripts/global_traj_sim/build_map.py <맵> 으로 raceline을 생성한 뒤
colcon build (raceline CSV만 패키지에 설치되고, 맵은 설치되지 않는다 —
launch가 <ws>/maps 를 직접 읽으므로 맵을 고쳤을 때 재빌드가 필요 없다).
`simm <맵>` 이 이 세 단계를 한 번에 한다. 다른 폴더를 쓰려면 maps_dir:=<경로>.
속도/마찰/여유는 build_map.py 상단 설정 블록에서 지정한다.
시작 포즈는 raceline CSV 첫 행에서, 상대 주차 위치는 맵 경계와 레이스라인에서
자동으로 잡는다 — 맵을 추가할 때 이 파일에 손댈 것이 없다.

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
import math
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

# 워크스페이스 공용 맵 폴더. mapping.launch.py(mapp)가 저장하고
# localization / raceline_tuner / build_map 이 전부 여기를 본다 — 맵의 단일 위치다.
# (planning_pkg/maps 사본은 폐지됐다: 같은 이름 맵이 두 벌 있으면 어느 쪽이
#  쓰였는지 조용히 갈린다. maps_dir:= 로 대회 맵 폴더를 가리킬 수 있다.)
WS_MAPS = os.path.expanduser('~/IFAC_ws/maps')

# 상대 차량 '주차장' (제거 시 이동 위치)은 맵마다 다르므로 이름별 표를 두지 않고
# 맵 경계 + 레이스라인에서 매번 계산한다 (_auto_park_pose). 맵 네 모서리 중
# 레이스라인에서 가장 먼 곳을 골라 맵 밖으로 이만큼 더 민다[m].
# 맵 밖 좌표는 gym 레이캐스트가 r=c=-1로 처리하므로(laser_models.xy_2_rc) 안전하고,
# 외벽에 가려 ego LiDAR에는 잡히지 않는다.
# (my_track 계열은 16x8m로 작아 LiDAR 사거리(≈30m) 밖으로는 못 뺀다 — 상대가
#  유령 장애물로 보이면 정적 장애물(Publish Point)로 회피를 시험할 것.)
PARK_MARGIN_M = 4.0


def _runnable_maps(maps_dir, gt_dir):
    """지금 그대로 돌릴 수 있는 맵 = maps_dir에 yaml이 있고 raceline CSV도 있는 것."""
    have_map = {
        os.path.splitext(os.path.basename(f))[0]
        for f in glob.glob(os.path.join(maps_dir, '*.yaml'))}
    have_csv = {
        os.path.basename(f).split('_iqp_')[0]
        for f in glob.glob(os.path.join(gt_dir, '*_iqp_*.csv')) if os.path.isfile(f)}
    return sorted(have_map & have_csv)


def _read_raceline(csv_path):
    """raceline CSV -> (ego 시작 포즈 (x, y, psi), 전체 (x, y) 목록).

    시작 포즈는 첫 행, (x,y) 목록은 상대 주차 위치를 자동으로 잡는 데 쓴다.
    """
    start = None
    points = []
    with open(csv_path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            cols = [float(c) for c in line.split(';')]
            if start is None:
                start = (cols[1], cols[2], cols[5])
            points.append((cols[1], cols[2]))
    if start is None:
        raise RuntimeError(f'raceline CSV가 비어 있음: {csv_path}')
    return start, points


def _map_bounds(map_yaml, image_path, fallback_points):
    """맵의 월드 좌표 경계 (xmin, ymin, xmax, ymax).

    nav2 맵 yaml의 origin은 이미지 좌하단의 월드 좌표다. 회전(origin[2])은
    이 스택의 SLAM 맵에서 항상 0이라 무시한다 — 0이 아니어도 경계가 헐거워질
    뿐이라 '맵 밖에 주차한다'는 결론은 안전한 쪽으로 유지된다.
    PIL을 못 쓰면 레이스라인 bbox로 폴백한다 (gym 자체가 PIL로 맵을 열므로
    실제로 걸릴 일은 없지만, 폴백이 있으면 launch가 죽지는 않는다).
    """
    try:
        from PIL import Image
        with open(map_yaml, encoding='utf-8') as handle:
            meta = yaml.safe_load(handle)
        resolution = float(meta['resolution'])
        origin_x, origin_y = float(meta['origin'][0]), float(meta['origin'][1])
        with Image.open(image_path) as img:
            width_px, height_px = img.size
        return (origin_x, origin_y,
                origin_x + width_px * resolution,
                origin_y + height_px * resolution)
    except Exception:                                   # noqa: BLE001
        xs = [p[0] for p in fallback_points]
        ys = [p[1] for p in fallback_points]
        return min(xs), min(ys), max(xs), max(ys)


def _auto_park_pose(bounds, raceline_xy):
    """상대 차량 주차 위치: 맵 네 모서리 중 레이스라인에서 가장 먼 곳을 골라
    맵 밖으로 PARK_MARGIN_M 만큼 더 민 좌표. -> ((x, y), 레이스라인까지 거리)"""
    xmin, ymin, xmax, ymax = bounds
    best_xy, best_clearance = None, -1.0
    for corner_x, corner_y in ((xmin, ymin), (xmin, ymax), (xmax, ymin), (xmax, ymax)):
        park_x = corner_x - PARK_MARGIN_M if corner_x == xmin else corner_x + PARK_MARGIN_M
        park_y = corner_y - PARK_MARGIN_M if corner_y == ymin else corner_y + PARK_MARGIN_M
        clearance = min(math.hypot(park_x - x, park_y - y) for x, y in raceline_xy)
        if clearance > best_clearance:
            best_xy, best_clearance = (park_x, park_y), clearance
    return best_xy, best_clearance


def _resolve_gym_map_image(map_yaml):
    """nav2 YAML의 image 필드를 따라 gym의 map_path/ext를 정확히 맞춘다."""
    with open(map_yaml, encoding='utf-8') as handle:
        metadata = yaml.safe_load(handle)
    image = metadata.get('image') if isinstance(metadata, dict) else None
    if not isinstance(image, str) or not image.strip():
        raise RuntimeError(f'맵 YAML에 유효한 image 항목이 없습니다: {map_yaml}')
    image_path = os.path.expanduser(image.strip())
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(map_yaml), image_path)
    image_path = os.path.abspath(image_path)
    image_stem, image_ext = os.path.splitext(image_path)
    if image_ext.lower() not in ('.png', '.pgm'):
        raise RuntimeError(
            f'gym이 지원하는 map image 확장자는 png/pgm입니다: {image_path}')
    if not os.path.isfile(image_path):
        raise RuntimeError(f'맵 YAML이 가리키는 image가 없습니다: {image_path}')
    return image_stem, image_ext


def launch_setup(context, *args, **kwargs):
    planning_share = get_package_share_directory('planning_pkg')
    gym_share = get_package_share_directory('f1tenth_gym_ros')

    params_file = os.path.join(planning_share, 'config', 'params.yaml')
    gym_config = os.path.join(gym_share, 'config', 'sim.yaml')

    map_name = LaunchConfiguration('map').perform(context).strip()
    maps_dir = os.path.expanduser(
        LaunchConfiguration('maps_dir').perform(context).strip() or WS_MAPS)
    gt_dir = os.path.join(planning_share, 'config', 'global_trajectory')
    if not map_name:
        # 기본 맵을 두지 않는다 — 옛 트랙 위에서 조용히 도는 것보다 죽는 게 낫다.
        raise RuntimeError(
            'map:=<맵이름> 이 필요합니다 (기본 맵 없음).\n'
            f'  바로 쓸 수 있는 맵: {", ".join(_runnable_maps(maps_dir, gt_dir)) or "(없음)"}\n'
            '  새 맵이면: simm <맵이름>  (CSV 생성 + 빌드 + 실행)')
    map_yaml = os.path.join(maps_dir, map_name + '.yaml')
    if not os.path.exists(map_yaml):
        available = sorted(
            os.path.splitext(os.path.basename(f))[0]
            for f in glob.glob(os.path.join(maps_dir, '*.yaml')))
        raise RuntimeError(
            f'맵 없음: {map_yaml}\n'
            f'  사용 가능한 맵: {", ".join(available) or "(없음)"}\n'
            '  맵은 워크스페이스 공용 폴더에 둔다 (mapp <이름>이 저장하는 곳). '
            '다른 폴더면 maps_dir:=<경로>')
    gym_map_stem, gym_map_ext = _resolve_gym_map_image(map_yaml)

    suffix = LaunchConfiguration('suffix').perform(context)
    if not suffix:
        # suffix 미지정 시 해당 맵의 iqp CSV를 찾아 접미사를 자동 추론한다.
        # 맵마다 raceline 파라미터(V_MAX/margin/mu)가 달라 하나의 고정 접미사로는
        # 안 맞기 때문. build_map.py가 생성 시점의 상수를 파일명에 박아둔다.
        # symlink-install 이 install/ 에 남긴 끊어진 링크는 제외한다. glob 은
        # 링크 대상을 확인하지 않아 지워진 옛 CSV까지 매칭하고, 그러면 아래
        # 중복 검사에 걸려 launch 가 통째로 죽는다.
        iqp_glob = sorted(
            p for p in glob.glob(os.path.join(gt_dir, f'{map_name}_iqp_*.csv'))
            if os.path.isfile(p))
        if not iqp_glob:
            raise RuntimeError(
                f'raceline 없음: {gt_dir}/{map_name}_iqp_*.csv '
                '(build_map.py로 생성 후 colcon build 필요)')
        if len(iqp_glob) > 1:
            raise RuntimeError(
                f'raceline이 여러 개라 임의 선택하지 않습니다: '
                f'{", ".join(os.path.basename(p) for p in iqp_glob)} '
                'suffix:=<정확한 생성 suffix>를 지정하세요')
        traj_csv = os.path.basename(iqp_glob[0])
        suffix = traj_csv[len(f'{map_name}_iqp_'):-len('.csv')]
    else:
        traj_csv = f'{map_name}_iqp_{suffix}.csv'
    expected_ot_csv = f'{map_name}_sp_{suffix}.csv'
    ot_csv = LaunchConfiguration('overtaking_csv_file').perform(context).strip()
    if ot_csv:
        if ot_csv != expected_ot_csv:
            raise RuntimeError(
                f'SP는 메인과 같은 map/suffix여야 합니다. 기대값: {expected_ot_csv}, '
                f'입력값: {ot_csv}')
        if not os.path.isfile(os.path.join(gt_dir, ot_csv)):
            raise RuntimeError(f'명시한 SP CSV가 없습니다: {os.path.join(gt_dir, ot_csv)}')
    center_csv = f'{map_name}_centerline.csv'
    traj_csv_path = os.path.join(gt_dir, traj_csv)
    if not os.path.isfile(traj_csv_path):
        raise RuntimeError(f'raceline CSV가 없습니다: {traj_csv_path}')
    (sx, sy, stheta), raceline_xy = _read_raceline(traj_csv_path)
    # 상대 주차 위치는 맵 경계 + 레이스라인에서 계산한다 (맵별 표 없음).
    (park_x, park_y), park_clearance = _auto_park_pose(
        _map_bounds(map_yaml, gym_map_stem + gym_map_ext, raceline_xy), raceline_xy)

    # gym 기본 설정 위에 맵과 시작 포즈를 덮어쓴다.
    # 상대(agent 2)는 주차장(레이스라인에서 30m+, ego LiDAR 밖)에서 시작 —
    # RViz "2D Goal Pose"로 트랙에 텔레포트하면 스폰된다.
    gym_overrides = {
        'map_path': gym_map_stem,
        'map_img_ext': gym_map_ext,
        'num_agent': 2,
        'sx': sx,
        'sy': sy,
        'stheta': stheta,
        'sx1': park_x,
        'sy1': park_y,
        'stheta1': 0.0,
    }

    # pf:=true — gym GT 대신 particle_filter로 위치추정한다.
    # gym_bridge는 map->ego_racecar/base_link(=완벽한 GT)를 직접 쏘므로 PF가
    # map->odom 보정을 끼워 넣을 자리가 없다. /tf를 /tf_gym으로 돌려서
    # sim_odom_drift_node가 그 레그만 걷어내고 드리프트 odom을 대신 세운다.
    use_pf = LaunchConfiguration('pf').perform(context).strip().lower()
    if use_pf not in ('true', 'false'):
        raise RuntimeError(f'pf:={use_pf!r} 는 true 또는 false여야 합니다')
    use_pf = (use_pf == 'true')

    # vehicle:=real_car — gym 차량 물리를 실차 실측값으로 덮어쓴다.
    # f110_gym 기본값은 CommonRoad 표준차라 실차와 크게 다르다. 특히 mu와 a_max가
    # 어긋나면 "고속에서 미끄러지는가"가 sim에서 전혀 재현되지 않는다.
    #   mu    1.0489 -> 0.90  : build_map.py MU (물리 노면 마찰).
    #                          (레이스라인 생성에 쓰는 MU_PROFILE 0.80은 계획용
    #                           derate라 다른 값 — 그건 CSV에 이미 반영돼 있다)
    #   a_max 9.51   -> 4.5   : deployment_profiles.A_ACC_MAX
    #   s_min -0.4189 -> -0.3578: deployment_profiles.MAX_RIGHT_STEER
    #   s_max  0.4189 ->  0.3800: deployment_profiles.MAX_LEFT_STEER
    #                           (실차의 방향별 control clamp와 동일)
    #   v_max 20.0   -> 8.5   : deployment_profiles.RACELINE_V_MAX
    #                          (2026-08-19: vesc.yaml speed_max 40000 / gain
    #                           4255.1 = 9.40 이 하드웨어 천장이라 8.5가 그
    #                           아래로 들어간다. 조향 LUT도 8.5까지 확장했지만
    #                           7.0 위는 이 차로 달려본 적 없는 추정 구간이다
    #                           — control real_car.yaml map/steering_lut_file
    #                           주석 참조)
    #   width/length 0.31/0.58 -> 0.30/0.55 : recovery/vehicle_{width,length}_m
    # 유지: lf+lr=0.3302 = 실차 wheelbase 0.33 / sv_max 3.2 = hardware
    #   max_servo_speed / m·I·h·C_Sf·C_Sr 는 실측이 없어 gym 기본값.
    #
    # ⚠️ 아래 숫자는 deployment_profiles.py 의 사본이다. 여기서 import 하지 않는
    #   이유는 그 모듈이 scripts/ 에만 있고 share/ 로 설치되지 않기 때문 —
    #   대신 test_safety_invariants.test_sim_gym_vehicle_matches_measured_plant
    #   가 AST 로 이 dict 를 읽어 드리프트를 막는다. deployment_profiles 를
    #   고치면 여기도 같이 고쳐야 테스트가 통과한다.
    #   (2026-08-07 동기화: a_max 2.2 -> 3.5, v_max 6.0 -> 7.0)
    #   (2026-08-10 동기화: a_max 3.5 -> 4.5, mu 0.75 -> 0.90. 0808~0810 튜닝에서
    #    deployment_profiles.A_ACC_MAX 와 build_map.MU 가 올라갔는데 sim 쪽 사본만
    #    옛 값으로 남아 있었다 — 실차가 달리는 my_track0809/0810 레이스라인은
    #    새 상수(A_ACC 4.5 / A_DEC_PLANNED 2.38 / MU_PROFILE 0.80)로 생성돼 있어
    #    gym 차량이 계획된 가속을 못 내고 코너에서 먼저 미끄러졌다.)
    #   (2026-08-11 동기화: s_max/s_min 0.36 -> 0.3578. gain 재적합(-0.8564)으로
    #    대칭 조향 한계가 0.357894 rad 로 내려갔는데 여기만 안 따라가 test_sim_gym_
    #    vehicle_matches_measured_plant 가 레드였다 — sim 이 실차보다 0.6% 더
    #    꺾을 수 있다고 믿은 채였다.)
    # ⚠️ gym의 accl_constraints는 가감속이 대칭(-a_max)이라 A_DEC_PLANNED 2.38을
    #   여기서 못 준다. control_pkg sim_real_car.yaml의 vehicle/max_decel_mps2
    #   2.38이 명령단에서 그 역할을 한다.
    REAL_CAR_VEHICLE = {
        'vehicle.mu': 0.90,
        'vehicle.a_max': 4.5,
        'vehicle.s_min': -0.3578, 'vehicle.s_max': 0.3800,
        'vehicle.v_max': 9.0, 'vehicle.v_min': -1.0,
        'vehicle.width': 0.30, 'vehicle.length': 0.55,
        # 실차 Hokuyo /scan 40 Hz (run_0804 실측 748msg/18.5s). gym 기본 250 Hz는
        # LiDAR 특성이 다를 뿐 아니라 /tf를 6배로 쏟아 구독자 tf 버퍼를 포화시켜
        # obstacle_detector가 전 스캔을 버린다. 물리 스텝(100 Hz)은 그대로다.
        'sensor_publish_rate_hz': 40.0,
        # 라이다 장착 위치 [m]. hardware sensors.yaml 의 base_to_laser_x 실측값.
        # gym 기본 0.275 와 5 mm 차이지만, 이건 진짜로 물리에 들어간다 —
        # gym_bridge 가 lidar_dist 로 넘겨 f110_gym 이 그 위치에서 레이를 쏜다.
        'scan_distance_to_base_link': 0.28,
        # ⚠️ 빔 수/FOV 는 여기서 못 맞춘다. 실차는 1081빔 · FOV 4.712389 rad
        #   (270.00°) 인데 f110_gym 은 RaceCar(num_beams=1080, fov=4.7) 로
        #   **하드코딩**돼 있고 f110_env / Simulator 둘 다 그 인자를 안 받는다
        #   (base_classes.py:69, f110_env.py:192). gym_config 의 scan_beams /
        #   scan_fov 는 ROS 메시지 헤더에만 쓰이므로, 여기서 1081/4.7124 를
        #   넣으면 **실제 레이와 어긋난 헤더**를 발행하게 된다 — 구독자가
        #   angle_min + i*inc 로 복원하는 점 위치가 통째로 틀어진다.
        #   남는 차이는 각분해능 0.0043559(sim) vs 0.00436332(실차) = 0.17%.
        #   4.5 m 거리의 0.30 m 차량이 sim 15.3점 / 실차 15.3점으로 같은 정수라
        #   min_cluster_size 같은 점수 문턱은 그대로 옮겨 쓸 수 있다.
        #   더 좁히려면 f110_gym 자체를 저장소로 벤더링해서 고쳐야 한다.
    }
    vehicle_profile = LaunchConfiguration('vehicle').perform(context).strip().lower()
    if vehicle_profile not in ('gym_default', 'real_car'):
        raise RuntimeError(
            f'vehicle:={vehicle_profile!r} 는 gym_default 또는 real_car여야 합니다')
    if vehicle_profile == 'real_car':
        gym_overrides.update(REAL_CAR_VEHICLE)

    bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        parameters=[gym_config, gym_overrides],
        remappings=[('/tf', '/tf_gym')] if use_pf else [],
    )
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[{'yaml_filename': map_yaml},
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
    if perception_mode not in ('ground_truth', 'lidar'):
        raise RuntimeError(
            f'perception:={perception_mode!r} 는 ground_truth | lidar 중 하나여야 합니다')
    for bool_arg in ('rviz', 'tracker'):
        value = LaunchConfiguration(bool_arg).perform(context).strip().lower()
        if value not in ('true', 'false'):
            raise RuntimeError(
                f'{bool_arg}:={value!r} 는 true 또는 false여야 합니다')
    lidar_perception = (perception_mode == 'lidar')

    # env:=sim_real_car 면 planner_stack이 environments/sim_real_car.yaml을
    # params.yaml 위에 덮어쓴다 (실차 거동 파라미터 + sim 배선).
    planner_env = LaunchConfiguration('env').perform(context).strip()
    stack_args = {
        # pf:=true 면 실차와 **같은** 모드('real')를 쓴다 — 둘 다 /pf/pose/odom을
        # 구독하고 이후 경로가 완전히 같다. 예전엔 이름만 다른 'sim_pf' 모드가
        # 있었지만 frenet_frame_node.cpp 가 그걸 제거하고 명시적으로 거부한다
        # (지정하면 std::invalid_argument 로 노드가 즉사한다).
        'odom_mode': 'real' if use_pf else 'sim_gt',
        'trajectory_csv_file': traj_csv,
        'overtaking_csv_file': ot_csv,
        'centerline_csv_file': center_csv,
        # recovery yaw 소스. pf:=false면 sim에 pf가 없으므로 gym GT odom을 쓴다.
        'pose_odom_topic': '/pf/pose/odom' if use_pf else '/ego_racecar/odom',
    }
    if lidar_perception:
        stack_args.update({
            'obstacle_topic': '/perception/obstacles_frenet',
            'bridge': 'true',
            'bridge_input_topic': '/perception/obstacles_detected',
            'map_filter': LaunchConfiguration('map_filter').perform(context),
        })
    if planner_env:
        stack_args['env'] = planner_env

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
    # 상대 예측기는 planner_stack 이 아니라 여기서 뜬다 (planner_stack 은 sim env
    # 에서 predictor:=auto -> off). 그래서 env 오버레이도 **여기서 직접** 얹어야
    # 한다 — 안 그러면 environments/<env>.yaml 의 predictor/* 가 sim 에서만
    # 조용히 무시되고, 실차와 다른 예측 파라미터로 동적 장애물을 검증하게 된다.
    predictor_params = [params_file]
    if planner_env:
        env_params = os.path.join(
            planning_share, 'config', 'environments', planner_env + '.yaml')
        if os.path.isfile(env_params):
            predictor_params.append(env_params)
    # 관측 소스는 인지 모드가 정한다. lidar 모드는 실차와 같은 자리
    # (/perception/obstacles_frenet = 브릿지 출력), ground_truth 모드는
    # virtual_obstacle_node 가 직접 쏘는 /perception/obstacles 다.
    # env 오버레이가 lidar 기준으로 적혀 있으므로 마지막에 명시적으로 덮는다
    # (예전엔 remap 으로 같은 일을 했는데, 오버레이가 들어오면 remap 은 param
    #  으로 바뀐 토픽 이름과 안 맞아 아무 것도 안 하게 된다).
    predictor_params.append({
        'predictor/obstacle_topic': ('/perception/obstacles_frenet'
                                     if lidar_perception
                                     else '/perception/obstacles'),
    })
    opponent_predictor_node = Node(
        package='planning_pkg',
        executable='opponent_predictor_node.py',
        name='opponent_predictor_node',
        parameters=predictor_params,
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(planning_share, 'rviz', 'sim_race.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # --- pf:=true 전용 노드 2개 ---
    pf_nodes = []
    if use_pf:
        pf_share = get_package_share_directory('particle_filter_cpp')
        pf_config = os.path.join(pf_share, 'config', 'localize_sim.yaml')
        if not os.path.isfile(pf_config):
            raise RuntimeError(
                f'PF 설정이 없습니다: {pf_config} '
                '(localization_ws를 빌드했는지 확인 — ./build_all.sh sim)')
        pf_nodes = [
            Node(
                package='planning_pkg',
                executable='sim_odom_drift_node.py',
                name='sim_odom_drift_node',
                parameters=[{
                    # 레이스라인 시작 포즈 = gym 스폰 위치. PF가 기동 시
                    # initialize_global()로 전역 산포하므로 좁은 트랙에서
                    # 수렴이 느리다 — 시작 포즈를 한 번 쏴서 잡아준다.
                    'drift/init_pose_x': sx,
                    'drift/init_pose_y': sy,
                    'drift/init_pose_yaw': stheta,
                }],
                output='screen',
            ),
            Node(
                package='particle_filter_cpp',
                executable='particle_filter_cpp_node',
                # 노드 이름은 'particle_filter' 그대로 — localize_sim.yaml 의
                # ros__parameters 키가 이 이름에 걸려 있고, C++ 판도 실차
                # localize_launch.py 에서 같은 이름으로 뜬다.
                name='particle_filter',
                parameters=[pf_config],
                # ⚠️ /clicked_point 충돌 회피 (sim 전용 리맵).
                # PF는 이 토픽으로 PointStamped를 받으면 initialize_global()
                # — 즉 파티클을 맵 전체에 다시 뿌린다 (C++ 판도 동일:
                # particle_filter_node.cpp 의 /clicked_point 구독).
                # 그런데 sim에서는 virtual_obstacle_node가 **같은 토픽**으로
                # 가상 장애물을 스폰한다(RViz "Publish Point"). 그대로 두면
                # 회피를 시험하려고 장애물을 찍는 순간 위치추정이 날아간다
                # — 실측: 클릭 직후 PF 오차 p50 4.5m / max 9.2m로 발산,
                # 그 결과 벽이 코리도 한복판에 얹혀 유령 장애물 14개까지 나오고
                # 차가 Trailing으로 정지했다. (장애물 없이 주행할 땐 75초 동안
                # PF 오차 p50 0.03m / 유령 0개로 멀쩡하다 — 원인은 이 충돌뿐)
                # PF 전역 재초기화가 필요하면 이 토픽으로 직접 쏘면 된다.
                # 실차 경로(localization_bringup)는 건드리지 않는다.
                remappings=[('/clicked_point', '/pf/relocalize_point')],
                output='screen',
            ),
        ]

    return [
        LogInfo(msg=f'[sim_race] map={map_name} raceline={traj_csv} '
                    f'start=({sx:.2f}, {sy:.2f}) '
                    f'상대주차=({park_x:.1f}, {park_y:.1f}) '
                    f'라인까지 {park_clearance:.1f} m'),
        bridge_node,
        map_server_node,
        nav_lifecycle_node,
        ego_robot_publisher,
        opp_robot_publisher,
        *pf_nodes,
        planner_stack,
        tracker_node,
        virtual_obstacle_node,
        opponent_driver_node,
        opponent_predictor_node,
        rviz_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        # 기본 맵 없음(필수 인자). 맵 이름을 여기에 박아두면 실차 트랙이 바뀔 때마다
        # 이 파일을 고쳐야 하고, 안 고치면 옛 트랙 위에서 조용히 돈다.
        DeclareLaunchArgument('map', default_value='',
                              description='맵 이름 (<maps_dir>/<이름>.yaml) — 필수. '
                                          '빈 값이면 사용 가능한 맵 목록을 띄우고 '
                                          '멈춘다. 원커맨드는 simm <맵이름>'),
        DeclareLaunchArgument('maps_dir', default_value='',
                              description=f'맵 폴더. 빈 값이면 {WS_MAPS} '
                                          '(mapp이 저장하는 워크스페이스 공용 폴더). '
                                          '대회 맵 폴더를 쓰려면 여기에 지정'),
        DeclareLaunchArgument('suffix', default_value='',
                              description='raceline CSV 접미사 (iqp/sp 공통). '
                                          '비워두면 맵의 iqp CSV에서 자동 추론 '
                                          '(형식: max<V>ms_margin<M>m_mu<MU>). '
                                          '한 맵에 여러 raceline이 있을 때만 지정.'),
        DeclareLaunchArgument(
            'overtaking_csv_file', default_value='',
            description='검증 완료한 SP CSV의 정확한 파일명. 기본 빈 값으로 '
                        'SP 자동 활성화를 금지하며, 메인과 같은 map/suffix만 허용.'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='RViz 실행 여부'),
        DeclareLaunchArgument('tracker', default_value='true',
                              description='Python 트래커 실행 여부 (false면 C++ control_node 사용)'),
        DeclareLaunchArgument('pf', default_value='false',
                              description='true면 gym GT 대신 particle_filter로 '
                                          '위치추정한다 (sim_odom_drift_node가 '
                                          '드리프트 odom 레그를 만들고 PF가 '
                                          'map->odom 보정). 실차와 같은 위치추정 '
                                          '오차 위에서 인지/플래닝을 검증할 때 사용. '
                                          'localization_ws 빌드 필요.'),
        DeclareLaunchArgument('env', default_value='',
                              description='planner 환경 프로파일 '
                                          '(planning config/environments/<env>.yaml). '
                                          'sim_real_car = 실차 거동 파라미터 + sim 배선. '
                                          '빈 값이면 params.yaml 기본값만 사용.'),
        DeclareLaunchArgument('vehicle', default_value='gym_default',
                              description='gym 차량 물리 프로파일: gym_default | real_car. '
                                          'real_car는 mu 0.90 / a_max 4.5 / 조향 좌 0.3800·우 -0.3578 / '
                                          'v_max 7.0 등 실측값을 넣어 고속 슬립까지 '
                                          '실차에 맞춘다.'),
        DeclareLaunchArgument('map_filter', default_value='',
                              description='bridge/use_map_filter override (true|false). '
                                          '맵 거리변환으로 벽 클러스터 제거. '
                                          'perception:=lidar + pf:=true 와 함께 쓰면 '
                                          '벽/장애물 구별을 A/B로 볼 수 있다.'),
        DeclareLaunchArgument('perception', default_value='ground_truth',
                              description='ground_truth: 가상 장애물 GT 직접 주입(기존), '
                                          'lidar: 실제 perception 체인 '
                                          '(/scan_overlay -> detector -> bridge -> spliner)'),
        OpaqueFunction(function=launch_setup),
    ])
