"""맵 작성 원커맨드 launch — 하드웨어 + slam_toolbox + 맵 저장을 한 번에.

  ros2 launch integration_pkg mapping.launch.py map_name:=내맵이름

이 한 줄이 아래를 띄운다:
  A) f1tenth_stack bringup_launch.py    (LiDAR, VESC, joy, mux, TF)
  B) slam_toolbox (async, mapping 모드)  — 5초 뒤 기동 (/scan·TF가 먼저 나오게)
  C) RViz                                — 맵이 그려지는 걸 보면서 주행
  D) map_saver_node                      — /map을 캐시했다가 파일로 저장

사람이 할 일: **조이스틱으로 트랙을 한 바퀴 천천히 돌고 Ctrl+C.**
Ctrl+C 하는 순간 맵이 `map_dir/map_name.{pgm,yaml}`로 저장된다.

저장 위치는 기본이 **~/IFAC_ws/maps/** 다 (워크스페이스 공용 맵 폴더).

주요 인자:
  map_name:=track1      저장할 맵 이름 (기본 my_map)
  map_dir:=<경로>       저장 디렉터리 (기본 ~/IFAC_ws/maps)
  hardware:=false       이미 A가 떠 있을 때
  rviz:=false           화면 없이
  save_on_exit:=false   Ctrl+C 자동 저장을 끄고 /save_map 서비스로만 저장
  overwrite:=false      이름이 이미 쓰일 때만 거부. 기본은 덮어쓰기 허용

이름이 겹치면 **주행 전에** 막는다: map_dir 은 localization / raceline_tuner /
build_map 모두가 먼저 뒤지는 폴더라, 기존 맵이나 패키지 맵(particle_filter/maps,
planning_pkg/maps)과 같은 이름으로 저장하면 그 맵이 에러 없이 조용히 가려진다.

주행 중 중간 저장:
  ros2 service call /save_map std_srvs/srv/Trigger

저장 후 그 맵으로 주행하려면 — **재빌드 불필요**:
  ros2 launch integration_pkg real_car.launch.py map:=track1
  (localization.launch.py 의 map:= 이 ~/IFAC_ws/maps 를 먼저 찾는다.
   단 planner raceline CSV는 별도로 step4_main.py 로 뽑아야 한다)

전제 source: ros humble -> integration_ws -> hardware_ws -> localization_ws
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# 워크스페이스 공용 맵 폴더. localization.launch.py 의 map:= 가 이 폴더를
# 먼저 뒤지므로, 여기에 저장하면 재빌드 없이 바로 map:=<이름> 으로 주행할 수 있다.
DEFAULT_MAP_DIR = os.path.expanduser('~/IFAC_ws/maps')


def _is_true(value):
    return str(value).lower() in ('true', '1', 'yes')


def _share(package):
    """워크스페이스를 source 안 했을 때 알아듣기 쉬운 메시지로 죽게 한다."""
    try:
        return get_package_share_directory(package)
    except PackageNotFoundError:
        raise RuntimeError(
            "'%s' 패키지를 찾을 수 없습니다. 해당 워크스페이스를 build/source "
            '했는지 확인하세요 (hardware_ws, localization_ws 포함).' % package)


# 이름이 겹치면 '가려지는' 패키지 맵 폴더들. localization / raceline_tuner /
# build_map 은 모두 map_dir(~/IFAC_ws/maps)을 먼저 뒤지므로, 여기 있는 맵과
# 같은 이름으로 저장하면 그 맵들이 조용히 안 쓰이게 된다.
SHADOWED_PACKAGES = (
    ('particle_filter', 'maps'),    # localization 이 2순위로 찾는 곳
    ('planning_pkg', 'maps'),       # raceline_tuner / build_map 이 2순위로 찾는 곳
)


def _name_conflicts(map_dir, map_name):
    """map_name 으로 저장하면 무엇을 덮어쓰거나 가리는지 목록으로 돌려준다.

    주행을 마친 뒤(Ctrl+C 저장 시점)에 알려주면 한 바퀴를 통째로 날리므로
    반드시 launch 시점에 검사한다. 패키지를 source 안 했으면 그 항목은
    검사에서 빠질 뿐, 매핑 자체를 막지는 않는다.
    """
    conflicts = []
    existing = os.path.join(map_dir, map_name + '.yaml')
    if os.path.isfile(existing):
        conflicts.append('덮어씀:   %s (기존 맵)' % existing)
    for package, subdir in SHADOWED_PACKAGES:
        try:
            path = os.path.join(get_package_share_directory(package), subdir,
                                map_name + '.yaml')
        except PackageNotFoundError:
            continue
        if os.path.isfile(path):
            conflicts.append('가려짐:   %s (%s 패키지 맵)' % (path, package))
    return conflicts


def launch_setup(context, *args, **kwargs):
    hardware = _is_true(LaunchConfiguration('hardware').perform(context))
    rviz = _is_true(LaunchConfiguration('rviz').perform(context))
    map_name = LaunchConfiguration('map_name').perform(context)
    map_dir = os.path.expanduser(LaunchConfiguration('map_dir').perform(context))
    save_on_exit = _is_true(LaunchConfiguration('save_on_exit').perform(context))
    overwrite = _is_true(LaunchConfiguration('overwrite').perform(context))

    conflicts = _name_conflicts(map_dir, map_name)
    if conflicts and not overwrite:
        raise RuntimeError(
            "map_name:='%s' 은 이미 쓰이는 이름입니다:\n"
            '    %s\n'
            '  %s 는 다른 폴더보다 먼저 검색되므로, 이 이름으로 저장하면 위 맵이\n'
            '  에러 없이 조용히 안 쓰이게 됩니다 (주행은 되는데 다른 맵으로 됩니다).\n'
            '  → 그냥 덮어쓰려면: mapp %s\n'
            '  → 정말 막고 싶을 때만: mapp %s overwrite:=false'
            % (map_name, '\n    '.join(conflicts), map_dir,
               map_name, map_name))

    actions = []
    if conflicts:
        actions.append(LogInfo(msg=(
            '\n[경고] 기존 이름 재사용 — 아래를 덮어쓰거나 가립니다:\n    %s\n'
            % '\n    '.join(conflicts))))

    if hardware:
        actions.append(LogInfo(msg='[1/3] 하드웨어 bringup (LiDAR/VESC/joy/mux)...'))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _share('f1tenth_stack') + '/launch/bringup_launch.py'),
        ))

    # /scan 과 base_link->laser TF 가 먼저 흐르게 잠깐 두고 SLAM 을 띄운다.
    # 바로 띄우면 "message filter dropping message" 경고로 로그가 도배된다.
    actions.append(TimerAction(period=5.0, actions=[
        LogInfo(msg='[2/3] slam_toolbox (mapping 모드) 시작...'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _share('localization_bringup') + '/launch/localization.launch.py'),
            launch_arguments={
                'backend': 'slam_toolbox',
                'mode': 'mapping',
                'use_sim_time': 'false',
                # localization.launch.py 의 rviz 인자는 particle_filter 전용이라
                # mapping 에서는 효과가 없다. RViz 는 아래에서 직접 띄운다.
                'rviz': 'false',
            }.items(),
        ),
    ]))

    if rviz:
        # pf.rviz 는 Map(/map) + LaserScan(/scan) + TF + Grid 를 이미 갖고 있어
        # 맵 작성 화면으로 그대로 쓸 수 있다. PF 전용 디스플레이(파티클 등)는
        # 토픽이 없어 비어 있을 뿐 문제되지 않는다.
        actions.append(TimerAction(period=6.0, actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2_mapping',
                output='log',
                arguments=['-d', PathJoinSubstitution([
                    FindPackageShare('particle_filter'), 'rviz', 'pf.rviz'])],
                parameters=[{'use_sim_time': False}],
            ),
        ]))

    actions.append(Node(
        package='integration_pkg',
        executable='map_saver_node.py',
        name='map_saver_node',
        output='screen',
        parameters=[{
            'map_dir': map_dir,
            'map_name': map_name,
            'save_on_exit': save_on_exit,
        }],
    ))

    actions.append(LogInfo(msg=(
        '\n'
        '=========================================================\n'
        ' [3/3] 맵 작성 준비 완료\n'
        '   1. 조이스틱으로 트랙을 천천히 한 바퀴 도세요\n'
        '      (RViz에서 맵이 채워지는 걸 확인하며 — 급회전은 정합 실패 유발)\n'
        '   2. 출발선으로 돌아와 루프가 닫히면 Ctrl+C\n'
        '   3. 저장 위치: %s\n'
        '   (중간 저장: ros2 service call /save_map std_srvs/srv/Trigger)\n'
        '=========================================================\n'
        % os.path.join(map_dir, map_name + '.{pgm,yaml}')
    )))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_name', default_value='my_map',
                              description='저장할 맵 이름 (확장자 없이)'),
        DeclareLaunchArgument('map_dir', default_value=DEFAULT_MAP_DIR,
                              description='맵을 저장할 디렉터리'),
        DeclareLaunchArgument('hardware', default_value='true',
                              description='f1tenth_stack bringup 실행 여부'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='맵 확인용 RViz 실행 여부'),
        DeclareLaunchArgument('save_on_exit', default_value='true',
                              description='Ctrl+C 시 자동 저장 여부'),
        DeclareLaunchArgument('overwrite', default_value='true',
                              description='기존 맵/패키지 맵과 이름이 겹쳐도 진행 '
                                          '(기본). false면 launch 단계에서 거부. '
                                          '겹친 파일은 저장 시 .bak 으로 백업된다'),
        OpaqueFunction(function=launch_setup),
    ])
