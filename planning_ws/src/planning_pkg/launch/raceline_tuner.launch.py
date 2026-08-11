"""RViz 인터랙티브 레이스라인 튜너 (HMCL raceline_tuner의 ROS2 구현).

맵 + 튜너 노드 + RViz를 띄운다. 코너별로 라인/속도를 드래그로 조절:
  ros2 launch planning_pkg raceline_tuner.launch.py map:=<맵이름>    # map은 필수
  ros2 launch planning_pkg raceline_tuner.launch.py map:=<맵이름> csv:=<다른 raceline>.csv

csv를 안 주면 map 이름으로 config/global_trajectory/<맵>_iqp_*.csv를 glob으로 찾는다
(planner_stack.launch.py와 같은 규칙). sp 라인을 편집하려면 csv:=<맵>_sp_*.csv로 직접 지정.

map:= 하나면 나머지는 전부 CSV에서 유도되므로 손으로 맞출 필요가 없다:
  v_max/margin/mu <- 파일명 (<맵>_iqp_max6.00ms_margin0.08m_mu0.75.csv)
  stride/falloff <- 트랙 전장 (마지막 행의 s_m)
셋 다 명시하면 명시값이 이긴다. v_max/mu를 잘못 주면 "속도 재계산"이 엉뚱한
프로파일을 만들기 때문에(README ⚠️ 1) 자동 유도가 기본이다.

배경 맵은 워크스페이스 공용 폴더 ~/IFAC_ws/maps 한 곳에서 찾는다
(build_map.py / localization.launch.py / sim_race.launch.py와 같은 위치).
mapping.launch.py(mapp)가 저장하는 곳이라 복사 없이 바로 쓴다. 따라서
`mapp <이름>` -> `mkcsv <이름>` -> `./build_all.sh` 뒤에 바로
`raceline_tuner.launch.py map:=<이름>` 이 뜬다.

주의: 튜너는 '소스 트리'의 CSV를 직접 편집한다 (config/global_trajectory/).
저장 후 colcon build 해야 런타임 노드에 반영된다.
"""

import glob
import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 소스 트리 경로 (튜너는 install이 아니라 소스 CSV를 편집해야 재빌드로 반영됨)
SRC_CONFIG = os.path.expanduser(
    '~/IFAC_ws/planning_ws/src/planning_pkg/config/global_trajectory')

# mapping.launch.py(mapp)가 맵을 저장하는 워크스페이스 공용 폴더 = 맵의 단일 위치.
# 맵은 패키지에 설치하지 않으므로 새 맵을 만들면 복사도 재빌드도 없이 바로 뜬다.
WS_MAPS = os.path.expanduser('~/IFAC_ws/maps')


def _resolve_csv(map_name):
    """map 이름 -> <맵>_iqp_*.csv 파일명. 소스 트리에서 찾는다."""
    found = sorted(glob.glob(os.path.join(SRC_CONFIG, map_name + '_iqp_*.csv')))
    if not found:
        available = sorted({os.path.basename(f).split('_iqp_')[0]
                            for f in glob.glob(os.path.join(SRC_CONFIG, '*_iqp_*.csv'))})
        raise RuntimeError(
            'map:=%s 의 raceline CSV(%s_iqp_*.csv)가 없습니다: %s\n'
            '  가능한 맵: %s\n'
            '  (csv:=<파일명>.csv 로 직접 지정할 수도 있습니다)'
            % (map_name, map_name, SRC_CONFIG, ', '.join(available) or '(없음)'))
    if len(found) > 1:
        raise RuntimeError(
            'map:=%s 에 raceline이 여러 개라 임의 선택하지 않습니다: %s\n'
            '  csv:=<정확한 파일명>.csv 를 지정하세요.'
            % (map_name, ', '.join(os.path.basename(path) for path in found)))
    return os.path.basename(found[0])


# mkcsv(build_map.py)가 파일명에 생성 파라미터를 그대로 박아 둔다:
#   <맵>_iqp_max6.00ms_margin0.08m_mu0.75.csv
# 그래서 "속도 재계산"이 써야 할 v_max/mu를 사람이 다시 입력할 필요가 없다.
_NUMBER = r'([0-9]+(?:\.[0-9]+)?)'
_CSV_PARAM_RE = re.compile(
    r'_max' + _NUMBER + r'ms_.*?margin' + _NUMBER +
    r'm_mu' + _NUMBER + r'\.csv$')


def _parse_csv_params(csv_name):
    """CSV 파일명 -> (v_max, corridor_margin, mu)."""
    match = _CSV_PARAM_RE.search(csv_name)
    if not match:
        return None, None, None
    return tuple(float(match.group(index)) for index in (1, 2, 3))


def _track_length(csv_path):
    """CSV 마지막 데이터 행의 s_m = 트랙 전장 [m]. 못 읽으면 None."""
    try:
        last = None
        with open(csv_path) as handle:
            for line in handle:
                line = line.strip()
                if line and not line.startswith('#'):
                    last = line
        return None if last is None else float(last.split(';')[0])
    except (OSError, ValueError, IndexError):
        return None


def _auto_stride(length_m):
    """핸들 간격 [wpt]. 간격 1.0~2.5 m 안에서 핸들이 ~30개 되도록.

    my_track(29.3 m) -> 1.0 m = 10 wpt, Spielberg(339 m) -> 2.5 m = 25 wpt.
    README의 권장값 두 개를 그대로 재현한다.
    """
    spacing_m = min(max(length_m / 30.0, 1.0), 2.5)
    return int(round(spacing_m / 0.1))


def _auto_falloff(length_m):
    """드래그 영향 반경 [m]. 전장의 10%, 3~8 m로 클램프.

    my_track(29.3 m) -> 3.0, Spielberg(339 m) -> 8.0. 좁은 트랙에서 ±8 m로
    끌면 트랙 절반이 딸려오는 문제(README ⚠️ 2)를 전장에 맞춰 자동으로 막는다.
    """
    return round(min(max(0.1 * length_m, 3.0), 8.0), 2)


def _resolve_map_yaml(map_name):
    """맵 이름 -> <맵>.yaml 절대경로. 워크스페이스 공용 맵 폴더 한 곳만 본다."""
    candidate = os.path.join(WS_MAPS, map_name + '.yaml')
    if os.path.isfile(candidate):
        return candidate
    available = sorted({os.path.splitext(os.path.basename(f))[0]
                        for f in glob.glob(os.path.join(WS_MAPS, '*.yaml'))})
    raise RuntimeError(
        '맵이 없습니다: %s.yaml\n'
        '  찾아본 곳: %s\n'
        '  가능한 맵: %s\n'
        '  (mapp <이름>으로 만든 맵이 바로 여기에 저장됩니다. 맵은 패키지에\n'
        '   설치되지 않으므로 맵을 추가/수정해도 colcon build는 필요 없습니다.)'
        % (map_name, WS_MAPS, ', '.join(available) or '(없음)'))


def launch_setup(context, *args, **kwargs):
    planning_share = get_package_share_directory('planning_pkg')
    map_name = LaunchConfiguration('map').perform(context).strip()
    if not map_name:
        available = sorted({os.path.splitext(os.path.basename(f))[0]
                            for f in glob.glob(os.path.join(WS_MAPS, '*.yaml'))})
        raise RuntimeError(
            'map:=<맵이름> 이 필요합니다 (기본 맵 없음).\n'
            '  가능한 맵: %s' % (', '.join(available) or '(없음)'))
    csv_name = LaunchConfiguration('csv').perform(context) or _resolve_csv(map_name)
    csv_path = os.path.join(SRC_CONFIG, csv_name)
    if not os.path.isfile(csv_path):
        raise RuntimeError('raceline CSV가 없습니다: %s' % csv_path)
    map_yaml = _resolve_map_yaml(map_name)
    print('[raceline_tuner] map=%s (%s)  csv=%s' % (map_name, map_yaml, csv_name))

    # --- 인자 자동 결정 -----------------------------------------------------
    # v_max/mu는 CSV 파일명이, stride/falloff는 트랙 전장이 정한다. 비워 두면
    # 여기서 유도하고, 명시하면 그 값이 이긴다. (map:= 하나로 뜨게 하는 핵심)
    tuner_params = {'tuner/raceline_csv': csv_path}

    def _explicit(name):
        raw = LaunchConfiguration(name).perform(context).strip()
        return float(raw) if raw else None

    csv_v_max, csv_margin, csv_mu = _parse_csv_params(csv_name)
    v_max = _explicit('v_max') or csv_v_max
    mu = _explicit('mu') or csv_mu
    if v_max is not None:
        tuner_params['tuner/v_max_mps'] = v_max
    if mu is not None:
        tuner_params['tuner/mu'] = mu
    if v_max is None or mu is None:
        # 노드 기본값(12.0 / 0.85)이 쓰인다. 라인 편집에는 영향이 없고
        # "속도 재계산" 메뉴에만 쓰이므로 띄우는 것 자체는 막지 않는다.
        print('[raceline_tuner] ⚠ 파일명에서 v_max/mu를 못 읽었습니다: %s\n'
              '   "속도 재계산"을 쓸 거라면 v_max:= / mu:= 를 직접 넘기세요.' % csv_name)
    explicit_margin = _explicit('corridor_margin')
    corridor_margin = (
        explicit_margin if explicit_margin is not None else csv_margin)
    if corridor_margin is not None:
        tuner_params['tuner/corridor_margin_m'] = corridor_margin
    else:
        print('[raceline_tuner] ⚠ 파일명에서 corridor margin을 못 읽어 '
              '보수적 노드 기본값 0.15 m를 씁니다')

    length_m = _track_length(csv_path)
    stride_raw = LaunchConfiguration('stride').perform(context).strip()
    falloff = _explicit('falloff')
    if length_m is None:
        # CSV를 못 읽었다면 인자 없이는 유도할 수 없다 -> 노드 기본값(25 / 8.0)
        print('[raceline_tuner] ⚠ 트랙 전장을 못 읽어 stride/falloff는 노드 기본값을 씁니다')
    else:
        stride_raw = stride_raw or str(_auto_stride(length_m))
        falloff = falloff if falloff is not None else _auto_falloff(length_m)
        print('[raceline_tuner] 전장 %.1f m -> stride %s wpt (%.1f m), falloff %.2f m'
              % (length_m, stride_raw, int(stride_raw) * 0.1, falloff))
    if stride_raw:
        tuner_params['tuner/handle_stride'] = int(stride_raw)
    if falloff is not None:
        tuner_params['tuner/falloff_window_m'] = falloff

    min_wall = _explicit('min_wall')
    if min_wall is not None:
        tuner_params['tuner/min_wall_dist_m'] = min_wall
    print('[raceline_tuner] v_max=%s m/s  margin=%s m  mu=%s  (%s)'
          % (v_max, corridor_margin, mu,
             '파일명에서 자동' if _explicit('v_max') is None else '인자로 지정'))

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[{'yaml_filename': map_yaml},
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
        parameters=[tuner_params],
        output='screen',
    )
    # rviz 설정은 sim_race와 통합 (튜너 단독 실행 시 gym 토픽 디스플레이는
    # 데이터 없음으로 비어 있을 뿐 문제없다)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_tuner',
        arguments=['-d', os.path.join(planning_share, 'rviz', 'sim_race.rviz')],
    )
    return [map_server_node, nav_lifecycle_node, tuner_node, rviz_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='',
            # 기본 맵 없음(필수). 이름을 박아두면 실차 트랙이 바뀔 때마다 여기를
            # 고쳐야 하고, 안 고치면 옛 트랙의 라인을 편집하게 된다.
            description='배경 맵 이름 (<이름>.yaml, 확장자 제외) — 필수. '
                        '~/IFAC_ws/maps 에서 찾는다'),
        DeclareLaunchArgument(
            'csv',
            default_value='',
            description='편집할 raceline CSV 파일명 (config/global_trajectory/). '
                        '비우면 <map>_iqp_*.csv를 자동으로 찾는다'),
        DeclareLaunchArgument(
            'stride',
            default_value='',
            description='핸들 간격 (wpt 수, 0.1m/wpt). 비우면 전장에서 자동 '
                        '(간격 1.0~2.5 m). my_track -> 10, Spielberg -> 25'),
        DeclareLaunchArgument(
            'falloff',
            default_value='',
            description='드래그 영향 반경 [m]. 비우면 전장의 10% (3~8 m 클램프). '
                        'my_track(29.3 m) -> 3.0, Spielberg(339 m) -> 8.0'),
        DeclareLaunchArgument(
            'v_max',
            default_value='',
            description='"속도 재계산" 상한 [m/s]. 비우면 CSV 파일명의 max<...>ms 에서 읽는다 '
                        '(= 그 라인을 뽑을 때 쓴 값). 웬만하면 넘기지 말 것'),
        DeclareLaunchArgument(
            'mu',
            default_value='',
            description='"속도 재계산" 마찰계수. 비우면 CSV 파일명의 mu<...> 에서 읽는다. '
                        '웬만하면 넘기지 말 것'),
        DeclareLaunchArgument(
            'min_wall',
            default_value='',
            description='저장되는 d_left/d_right의 최소 여유 [m]. 비우면 0.20. '
                        '실제 벽 거리는 여기에 corridor_margin이 더해진다'),
        DeclareLaunchArgument(
            'corridor_margin',
            default_value='',
            description='map raycast 벽 거리에서 뺄 reserve [m]. 비우면 CSV '
                        '파일명의 margin<...>m에서 읽고, 없으면 보수적 기본 0.15'),
        OpaqueFunction(function=launch_setup),
    ])
