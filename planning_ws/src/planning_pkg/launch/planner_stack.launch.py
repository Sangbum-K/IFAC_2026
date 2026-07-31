"""raceline 추종 파이프라인 launch.

global_trajectory_node -> frenet_frame_node -> local_trajectory_node
                                            -> spliner_node (장애물 회피)
                                            -> recovery_supervisor_node
                                            -> planner_visualizer_node

파라미터는 config/params.yaml에서 로드하고, 자주 바꾸는 것만 CLI 인자로
override 가능:

  ros2 launch planning_pkg planner_stack.launch.py
  ros2 launch planning_pkg planner_stack.launch.py odom_mode:=real
  ros2 launch planning_pkg planner_stack.launch.py map:=my_track2
  ros2 launch planning_pkg planner_stack.launch.py \
      trajectory_csv_file:=<다른 raceline>.csv centerline_csv_file:=<맵>_centerline.csv

map:=<이름> 은 메인 raceline과 centerline을 이름으로 바꾸는 지름길이다. 파일명에
속도/마진/mu가 붙어 있어(<맵>_iqp_max6.00ms_margin0.08m_mu0.75.csv) 이름만으로는
유도할 수 없으므로 glob으로 찾는다:
  config/global_trajectory/<맵>_iqp_*.csv   raceline (필수)
  config/global_trajectory/<맵>_sp_*.csv    SP 추월라인 (자동 선택 금지, 명시 opt-in)
  config/centerline/<맵>_centerline.csv     시각화용
개별 *_csv_file 인자를 같이 주면 그쪽이 우선한다.

(기존 planner_pkg_launch.py를 대체: 존재하지 않는 planner_config.yaml 참조와
trajectory 파일명에서 centerline 이름을 소수점 파싱으로 유도하던 버그를 제거하고,
centerline은 명시적 인자/params.yaml 기본값으로만 정한다.)
"""

import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_map_csvs(share_dir, map_name):
    """map 이름 -> (raceline, sp, centerline) 파일명. 노드가 config/ 하위 상대경로로
    여는 값이므로 basename만 돌려준다. 못 찾으면 (필수 항목만) 알아듣게 죽는다."""
    traj_dir = os.path.join(share_dir, 'config', 'global_trajectory')
    center_dir = os.path.join(share_dir, 'config', 'centerline')
    notes = []

    def pick(directory, pattern):
        found = sorted(glob.glob(os.path.join(directory, pattern)))
        if len(found) > 1:
            raise RuntimeError(
                '여러 CSV가 %s에 매칭됩니다. 고속 주행에서 임의 선택하지 않습니다: %s\n'
                '  trajectory_csv_file:=<정확한 파일명>으로 지정하세요.'
                % (pattern, ', '.join(os.path.basename(f) for f in found)))
        return os.path.basename(found[0]) if found else None

    raceline = pick(traj_dir, map_name + '_iqp_*.csv')
    if raceline is None:
        available = sorted({
            os.path.basename(f).split('_iqp_')[0]
            for f in glob.glob(os.path.join(traj_dir, '*_iqp_*.csv'))})
        raise RuntimeError(
            "map:=%s 의 raceline CSV(%s_iqp_*.csv)가 없습니다: %s\n"
            '  사용 가능한 맵: %s\n'
            '  (레이스라인을 아직 안 뽑았다면 '
            'scripts/global_traj_sim/build_map.py <맵이름> 먼저)'
            % (map_name, map_name, traj_dir, ', '.join(available) or '(없음)'))

    # 파일 존재/같은 suffix만으로는 SP의 폐루프 안전 검증을 증명하지 못한다.
    # 현재 저장된 구형 SP 중에도 새 조향·마찰 검증을 실패하는 파일이 있으므로
    # map:= 자동 선택에서는 항상 끈다. 검증된 파일만 overtaking_csv_file:= 로
    # 명시해야 하며, global_trajectory_node가 기하/동역학/메인 호환성을 재검증한다.
    sp_candidate = raceline.replace('_iqp_', '_sp_', 1)
    sp = ''
    if os.path.isfile(os.path.join(traj_dir, sp_candidate)):
        notes.append(
            'SP(%s)는 자동 활성화하지 않음 — 검증 후 '
            'overtaking_csv_file:= 로 명시하세요' % sp_candidate)
    else:
        notes.append(
            '메인과 suffix가 같은 SP(%s) 없음 — 메인 라인만 씁니다'
            % sp_candidate)

    centerline = map_name + '_centerline.csv'
    if not os.path.isfile(os.path.join(center_dir, centerline)):
        notes.append('centerline(%s) 없음 — 시각화만 비게 됩니다' % centerline)

    return raceline, sp, centerline, notes


def launch_setup(context, *args, **kwargs):
    share_dir = get_package_share_directory('planning_pkg')
    params_file = os.path.join(share_dir, 'config', 'params.yaml')

    # 환경 프로파일: params.yaml 위에 environments/<env>.yaml을 덮어쓴다.
    # 실차: env:=real_car (frenet odom_mode real, 브릿지 토픽 전환 포함)
    env = LaunchConfiguration('env').perform(context)
    env_file = None
    if env:
        env_file = os.path.join(share_dir, 'config', 'environments', env + '.yaml')
        if not os.path.isfile(env_file):
            raise RuntimeError(f'environments/{env}.yaml 없음: {env_file}')

    # CLI override: 값이 주어진 것만 params.yaml 위에 덮어쓴다
    overrides = {
        'global_trajectory_node': {},
        'frenet_frame_node': {},
        'local_trajectory_node': {},
        'spliner_node': {},
        'recovery_supervisor_node': {},
        'opponent_predictor_node.py': {},
        'planner_visualizer_node': {},
    }
    # map:=<이름> — CSV 3종을 이름 하나로 해석한다. 개별 *_csv_file 인자가
    # 같이 오면 그쪽이 우선.
    map_name = LaunchConfiguration('map').perform(context)
    trajectory_arg = LaunchConfiguration('trajectory_csv_file').perform(context)
    map_traj = map_sp = map_center = None
    map_notes = []
    if map_name:
        if trajectory_arg:
            # 정확한 메인 CSV가 명시됐다면 같은 맵의 여러 IQP 후보를 다시
            # glob하다가 모호성 오류를 내지 않는다. map:=은 centerline 선택에만 쓴다.
            map_center = map_name + '_centerline.csv'
            if not os.path.isfile(os.path.join(
                    share_dir, 'config', 'centerline', map_center)):
                map_notes.append(
                    'centerline(%s) 없음 — 시각화만 비게 됩니다' % map_center)
        else:
            map_traj, map_sp, map_center, map_notes = \
                _resolve_map_csvs(share_dir, map_name)

    traj_csv = trajectory_arg or map_traj
    if traj_csv:
        overrides['global_trajectory_node'][
            'global_trajectory/trajectory_csv_file'] = traj_csv
    ot_csv = LaunchConfiguration('overtaking_csv_file').perform(context)
    if ot_csv:
        overrides['global_trajectory_node'][
            'global_trajectory/overtaking_csv_file'] = ot_csv
    elif trajectory_arg:
        # 메인 라인만 바꿨을 때 params.yaml의 이전 SP가 남지 않게 명시적으로
        # 비활성화한다. suffix가 같아도 검증 증거가 아니므로 자동 pairing 금지.
        overrides['global_trajectory_node'][
            'global_trajectory/overtaking_csv_file'] = ''
        map_notes.append(
            '메인 CSV override: SP 자동 선택을 금지해 추월선을 비활성화합니다')
    elif map_sp is not None:
        # 빈 문자열도 반드시 덮어써야 한다 — 안 그러면 params.yaml에 남아 있는
        # '이전 맵'의 SP 라인을 새 맵 위에서 그대로 읽는다.
        overrides['global_trajectory_node'][
            'global_trajectory/overtaking_csv_file'] = map_sp
    odom_mode = LaunchConfiguration('odom_mode').perform(context)
    if odom_mode:
        overrides['frenet_frame_node']['frenet_frame/odom_mode'] = odom_mode
    pose_topic = LaunchConfiguration('pose_odom_topic').perform(context)
    if pose_topic:
        overrides['recovery_supervisor_node'][
            'recovery/pose_odom_topic'] = pose_topic
    center_csv = LaunchConfiguration('centerline_csv_file').perform(context) or map_center
    if center_csv:
        overrides['planner_visualizer_node'][
            'planning_viz/centerline_csv_file'] = center_csv
    # 장애물 입력 토픽 override (spliner/recovery 공통).
    # 통합 sim에서 실제 perception 체인(/perception/obstacles_frenet)으로 전환할 때 사용.
    obstacle_topic = LaunchConfiguration('obstacle_topic').perform(context)
    if obstacle_topic:
        overrides['spliner_node']['spliner/obstacle_topic'] = obstacle_topic
        overrides['recovery_supervisor_node'][
            'recovery/obstacle_topic'] = obstacle_topic
        overrides['opponent_predictor_node.py'][
            'predictor/obstacle_topic'] = obstacle_topic

    def make_node(executable):
        params = [params_file]
        if env_file:
            params.append(env_file)
        if overrides.get(executable):
            params.append(overrides[executable])
        return Node(
            package='planning_pkg',
            executable=executable,
            name=executable.removesuffix('.py'),
            parameters=params,
            output='screen',
        )

    # Recovery 출력은 control_node의 lease 기반 override로 연결된다. 기본 기동해
    # perception/localization 미준비와 stale 입력을 fail-closed로 처리한다.
    def strict_bool(name):
        value = LaunchConfiguration(name).perform(context).strip().lower()
        if value not in ('true', 'false'):
            raise RuntimeError(
                f'{name}:={value!r} 는 유효하지 않습니다 (true 또는 false)')
        return value == 'true'

    recovery_on = strict_bool('recovery')

    nodes = [
        make_node('global_trajectory_node'),
        make_node('frenet_frame_node'),
        make_node('local_trajectory_node'),
        make_node('spliner_node'),
        make_node('planner_visualizer_node'),
    ]
    if recovery_on:
        # CLI에서 끄지 않은 한 내부 게이트도 함께 활성화한다.
        overrides['recovery_supervisor_node']['recovery/enabled'] = True
        nodes.insert(4, make_node('recovery_supervisor_node'))
    else:
        nodes.insert(0, LogInfo(msg='[planner_stack] 경고: recovery supervisor 비활성 '
                                    '— planner fault는 control stop으로 전달되지 않습니다'))

    # 실차 인터페이스 브릿지 (sim은 virtual_obstacle/gym GT가 대체):
    #  - perception 팀 laser-frame 장애물 -> frenet (/perception/obstacles_frenet)
    # /pf/pose/odom은 localization_ws particle_filter가 직접 발행 (publish_odom: 1)
    # — 플래닝 쪽에서 별도 pose 브릿지를 띄우지 않는다 (이중 발행 금지).
    bridge = strict_bool('bridge')
    if env in ('real_car', 'race_map') or bridge:
        bridge_input = LaunchConfiguration('bridge_input_topic').perform(context)
        if bridge_input:
            overrides['perception_obstacle_bridge_node.py'] = {
                'bridge/input_topic': bridge_input}
        nodes.append(make_node('perception_obstacle_bridge_node.py'))

    predictor_mode = LaunchConfiguration('predictor').perform(context).lower()
    if predictor_mode not in ('auto', 'true', 'false'):
        raise RuntimeError(
            f'predictor:={predictor_mode!r} 는 auto | true | false 중 하나여야 합니다')
    predictor_on = predictor_mode == 'true' or (
        predictor_mode == 'auto' and env in ('real_car', 'race_map'))
    if predictor_on:
        # 실차 raw /perception/obstacles는 메시지 타입이 다르므로 반드시 bridge
        # 출력인 planning ObstacleArray를 받는다.
        if not obstacle_topic and env in ('real_car', 'race_map'):
            overrides['opponent_predictor_node.py'][
                'predictor/obstacle_topic'] = '/perception/obstacles_frenet'
        nodes.append(make_node('opponent_predictor_node.py'))

    info = []
    if map_name:
        # 어떤 CSV가 실제로 걸렸는지 남긴다 — 맵을 바꿨는데 옛 라인이 물려
        # 있으면 주행 중엔 알아채기 어렵다.
        selected_sp = overrides['global_trajectory_node'].get(
            'global_trajectory/overtaking_csv_file', '')
        info.append('map:=%s -> raceline=%s, sp=%s, centerline=%s'
                    % (map_name, traj_csv, selected_sp or '(없음)', center_csv))
    # trajectory_csv_file 단독 override 경고도 map:= 없이 반드시 보이게 한다.
    info += ['  주의: ' + n for n in map_notes]
    if info:
        nodes.insert(0, LogInfo(msg='\n'.join(info)))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('env', default_value='',
                              description='환경 프로파일 (config/environments/<env>.yaml): '
                                          '빈 값=sim 기본, real_car | race_map | mapping'),
        DeclareLaunchArgument('map', default_value='',
                              description='맵 이름. CSV 3종(raceline/SP/centerline)을 '
                                          '이름으로 한꺼번에 해석한다. 빈 값이면 '
                                          'params.yaml의 값을 그대로 사용'),
        DeclareLaunchArgument('trajectory_csv_file', default_value='',
                              description='raceline CSV 파일명 (config/global_trajectory/)'),
        DeclareLaunchArgument('overtaking_csv_file', default_value='',
                              description='SP 추월 라인 CSV 파일명 (config/global_trajectory/)'),
        DeclareLaunchArgument('odom_mode', default_value='',
                              description='sim_gt | sim_pf | real'),
        DeclareLaunchArgument('centerline_csv_file', default_value='',
                              description='centerline CSV 파일명 (config/centerline/)'),
        DeclareLaunchArgument('pose_odom_topic', default_value='',
                              description='recovery yaw 소스 odom 토픽 '
                                          '(실차 /pf/pose/odom, sim /ego_racecar/odom)'),
        DeclareLaunchArgument('recovery', default_value='false',
                              description='recovery_supervisor_node 기동 여부. '
                                          '기본 false — 필요 시 recovery:=true로 opt-in '
                                          '(control의 lease 기반 safety override와 연결)'),
        DeclareLaunchArgument('predictor', default_value='auto',
                              description='opponent predictor: auto | true | false. '
                                          'auto는 real_car/race_map에서 기동'),
        DeclareLaunchArgument('obstacle_topic', default_value='',
                              description='spliner/recovery 장애물 토픽 override '
                                          '(통합 sim: /perception/obstacles_frenet)'),
        DeclareLaunchArgument('bridge', default_value='false',
                              description='perception_obstacle_bridge 강제 실행 '
                                          '(real_car/race_map env는 항상 실행)'),
        DeclareLaunchArgument('bridge_input_topic', default_value='',
                              description='브릿지 입력 토픽 override '
                                          '(통합 sim: /perception/obstacles_detected)'),
        OpaqueFunction(function=launch_setup),
    ])
