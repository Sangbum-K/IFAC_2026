"""RViz 인터랙티브 레이스라인 튜너 (HMCL raceline_tuner의 ROS2 구현).

맵 + 튜너 노드 + RViz를 띄운다. 코너별로 라인/속도를 드래그로 조절:
  ros2 launch planning_pkg raceline_tuner.launch.py                  # 기본 = my_track
  ros2 launch planning_pkg raceline_tuner.launch.py map:=Spielberg_map
  ros2 launch planning_pkg raceline_tuner.launch.py csv:=<다른 raceline>.csv

기본값은 대회맵 my_track 기준이다 (전장 29.3 m, 6 m/s · mu 0.75로 뽑은 라인).
csv를 안 주면 map 이름으로 config/global_trajectory/<맵>_iqp_*.csv를 glob으로 찾는다
(planner_stack.launch.py와 같은 규칙). sp 라인을 편집하려면 csv:=<맵>_sp_*.csv로 직접 지정.

주의: 튜너는 '소스 트리'의 CSV를 직접 편집한다 (config/global_trajectory/).
저장 후 colcon build 해야 런타임 노드에 반영된다.
"""

import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 소스 트리 경로 (튜너는 install이 아니라 소스 CSV를 편집해야 재빌드로 반영됨)
SRC_CONFIG = os.path.expanduser(
    '~/IFAC_ws/planning_ws/src/planning_pkg/config/global_trajectory')


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
    return os.path.basename(found[0])


def launch_setup(context, *args, **kwargs):
    planning_share = get_package_share_directory('planning_pkg')
    map_name = LaunchConfiguration('map').perform(context)
    csv_name = LaunchConfiguration('csv').perform(context) or _resolve_csv(map_name)
    csv_path = os.path.join(SRC_CONFIG, csv_name)
    if not os.path.isfile(csv_path):
        raise RuntimeError('raceline CSV가 없습니다: %s' % csv_path)
    map_stem = os.path.join(planning_share, 'maps', map_name)
    if not os.path.isfile(map_stem + '.yaml'):
        raise RuntimeError(
            '맵이 없습니다: %s.yaml\n'
            '  맵은 심링크가 아니라 빌드 시 복사됩니다 — src/planning_pkg/maps/에 넣고\n'
            '  colcon build --packages-select planning_pkg 하세요.' % map_stem)
    print('[raceline_tuner] map=%s  csv=%s' % (map_name, csv_name))

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
    getf = lambda n: float(LaunchConfiguration(n).perform(context))
    tuner_node = Node(
        package='planning_pkg',
        executable='raceline_tuner_node.py',
        name='raceline_tuner_node',
        parameters=[{'tuner/raceline_csv': csv_path},
                    {'tuner/handle_stride': int(LaunchConfiguration('stride').perform(context))},
                    {'tuner/falloff_window_m': getf('falloff')},
                    {'tuner/min_wall_dist_m': getf('min_wall')},
                    {'tuner/v_max_mps': getf('v_max')},
                    {'tuner/mu': getf('mu')}],
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
            default_value='my_track',
            description='배경 맵 이름 (maps/<이름>.yaml, 확장자 제외). 기본 = 대회맵'),
        DeclareLaunchArgument(
            'csv',
            default_value='',
            description='편집할 raceline CSV 파일명 (config/global_trajectory/). '
                        '비우면 <map>_iqp_*.csv를 자동으로 찾는다'),
        DeclareLaunchArgument(
            'stride',
            default_value='10',
            description='핸들 간격 (wpt 수, 0.1m/wpt). 기본 10 = 1.0 m (my_track 기준). '
                        '큰 트랙이면 25(2.5 m) 정도'),
        DeclareLaunchArgument(
            'falloff',
            default_value='3.0',
            description='드래그 영향 반경 [m]. 기본 3.0 (my_track 전장 29.3 m 기준). '
                        '큰 트랙이면 8.0'),
        DeclareLaunchArgument(
            'v_max',
            default_value='6.0',
            description='"속도 재계산" 상한 [m/s]. 편집할 raceline을 뽑을 때 쓴 값과 맞출 것 '
                        '(my_track = 6.0)'),
        DeclareLaunchArgument(
            'mu',
            default_value='0.75',
            description='"속도 재계산" 마찰계수. raceline 생성값과 맞출 것 (my_track = 0.75)'),
        DeclareLaunchArgument(
            'min_wall',
            default_value='0.35',
            description='라인-벽 최소 여유 [m]. CSV의 d_left/d_right는 이미 margin 0.08 m를 '
                        '뺀 값이므로 실제 여유는 여기에 +0.08 m'),
        OpaqueFunction(function=launch_setup),
    ])
