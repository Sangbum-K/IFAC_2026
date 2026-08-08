"""실차 원커맨드 launch — 하드웨어 → localization(+RViz) → 자율 스택을 자동 순서로.

  ros2 launch integration_pkg real_car.launch.py

기존에 터미널 3개로 나눠 하던 A/B/C를 하나로 합친 것이다. 사람이 할 일은
RViz에서 "2D Pose Estimate"로 초기 위치를 한 번 찍는 것뿐이고, 파티클이
수렴하면 자율 스택이 알아서 뜬다.

  A) f1tenth_stack bringup_launch.py       (LiDAR, VESC, joy, mux, TF)
  B) localization_bringup localization.launch.py backend:=particle_filter
                                            — 5초 뒤 기동 (/scan·/odom이 먼저 나오게)
  M) monitor_pkg monitor.launch.py env:=real_car
     초기 위치 지정 + 주행 모니터를 겸하는 RViz 창 하나. B와 같이 뜬다.
     (pf.rviz는 안 띄운다 — monitor.rviz에 파티클/InferredPose를 병합해 뒀다)
  G) wait_for_localization                  — 파티클 수렴까지 대기하는 게이트
  C) integration_pkg stack.launch.py env:=real_car   — G가 통과하면 자동 실행

맵을 바꾸려면 (PF 맵 + planner CSV 3종이 한 번에 따라간다):
  ros2 launch integration_pkg real_car.launch.py map:=my_track2

레이스라인 튜너로 손본 라인으로 달리려면 맵 이름에 _tuned 를 붙인다:
  ros2 launch integration_pkg real_car.launch.py map:=my_track0808_tuned
  → PF 맵/센터라인은 my_track0808 을 그대로 쓰고, 레이스라인만
    my_track0808_tuned_iqp_*.csv 로 바뀐다. 그 라인은 마찰원/조향 한계를 넘을 수
    있으므로 global_trajectory_node 가 advisory 모드로 뜬다 (경고만, 거부 안 함).

부분 실행이 필요하면 인자로 끌 수 있다:
  hardware:=false      이미 A가 떠 있을 때
  localization:=false  이미 B가 떠 있을 때
  rviz:=false          화면 없이
  autostart:=false     C를 자동으로 안 띄우고, 수렴만 확인하고 끝
  wait_timeout:=120.0  이 시간(초) 안에 수렴 못 하면 C를 띄우지 않음 (0=무한대기)

전제 source: ros humble -> planning_ws -> perception_ws -> control_ws
  -> integration_ws -> hardware_ws -> localization_ws  (sim_ws는 source 금지)
"""

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# 라인 변형 접미사 — planning_pkg/launch/planner_stack.launch.py 의 LINE_VARIANTS 와
# 같은 목록이어야 한다. 여기서 필요한 이유는 localization(PF)에 넘길 맵 이름에서
# 접미사를 먼저 떼야 하기 때문이다 (<맵>_tuned.yaml 은 존재하지 않는다).
# 두 launch 는 서로 다른 워크스페이스라 import 할 수 없어 사본으로 둔다.
# test/test_line_variant_routing.py 가 두 사본의 동기화를 강제한다.
LINE_VARIANTS = ('tuned',)


def split_line_variant(name):
    """'my_track0808_tuned' -> ('my_track0808', 'tuned'), 'my_track0808' -> (..., '')."""
    for variant in LINE_VARIANTS:
        suffix = '_' + variant
        if name.endswith(suffix) and len(name) > len(suffix):
            return name[:-len(suffix)], variant
    return name, ''


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


def launch_setup(context, *args, **kwargs):
    hardware = _is_true(LaunchConfiguration('hardware').perform(context))
    localization = _is_true(LaunchConfiguration('localization').perform(context))
    rviz = LaunchConfiguration('rviz').perform(context)
    autostart = _is_true(LaunchConfiguration('autostart').perform(context))
    wait_timeout = LaunchConfiguration('wait_timeout').perform(context)
    # 빈 값이면 각 하위 launch의 기존 기본값(localize.yaml / params.yaml)을 쓴다.
    map_name = LaunchConfiguration('map').perform(context)
    # map:=<맵>_tuned 는 '다른 맵'이 아니라 '같은 맵의 사람 튜닝 라인'이다.
    # localization(PF)은 <맵>.yaml 만 있으므로 접미사를 떼고 넘겨야 한다.
    # 레이스라인 CSV 선택은 planner_stack.launch.py 가 접미사를 보고 처리하므로
    # 자율 스택에는 map_name 을 그대로 넘긴다.
    map_base, line_variant = split_line_variant(map_name)

    actions = []

    if line_variant:
        actions.append(LogInfo(msg=(
            '[real_car] 라인 변형 "%s" — PF 맵/센터라인은 "%s" 를 쓰고 '
            '레이스라인만 "%s_iqp_*.csv" 를 씁니다.\n'
            '           이 라인은 사람이 손으로 튜닝한 것이라 마찰원/조향 한계를 '
            '넘을 수 있고, global_trajectory_node 가 advisory 로 뜹니다.\n'
            '           원본으로 달리려면 map:=%s (접미사 없이).'
            % (line_variant, map_base, map_name, map_base))))

    if hardware:
        actions.append(LogInfo(msg='[1/3] 하드웨어 bringup (LiDAR/VESC/joy/mux)...'))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _share('f1tenth_stack') + '/launch/bringup_launch.py'),
        ))

    if localization:
        # /scan·/odom이 먼저 흐르게 잠깐 두고 PF를 띄운다. PF는 시작 직후
        # laser->base_link TF를 조회하는데, 그게 없으면 초기 포즈 클릭이 무시된다.
        # RViz는 localization 쪽(pf.rviz)이 아니라 monitor_pkg가 띄운다 — 초기
        # 위치 지정과 주행 모니터를 한 창에서 하려고 monitor.rviz에 PF 파티클
        # 디스플레이를 병합해 뒀다. 여기서 rviz:=false로 pf.rviz를 막지 않으면
        # 창이 두 개 뜬다.
        loc_args = {
            'backend': 'particle_filter',
            'mode': 'localization',
            'use_sim_time': 'false',
            'rviz': 'false',
        }
        if map_base:
            loc_args['map'] = map_base
        actions.append(TimerAction(period=5.0, actions=[
            LogInfo(msg='[2/3] localization (particle_filter) + 모니터 RViz...'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _share('localization_bringup') + '/launch/localization.launch.py'),
                launch_arguments=loc_args.items(),
            ),
        ]))

    # 모니터 RViz + monitor_node. localization과 같은 타이밍에 띄운다 (PF가
    # 파티클을 뿌린 직후부터 초기 위치를 찍을 수 있어야 하므로).
    # 랩타임/CTE 오버레이는 자율 스택(C)이 뜬 뒤부터 채워진다.
    actions.append(TimerAction(period=5.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _share('monitor_pkg') + '/launch/monitor.launch.py'),
            launch_arguments={'env': 'real_car', 'rviz': rviz}.items(),
        ),
    ] + ([
        LogInfo(msg=(
            '\n'
            '=========================================================\n'
            ' 파티클 필터 초기 위치를 지정하세요 (모니터 RViz 창):\n'
            '  1. 차를 맵상 아는 지점(보통 출발선)에 실제로 갖다 놓기\n'
            '  2. 상단 툴바 "2D Pose Estimate" 클릭\n'
            '  3. 맵에서 차가 있는 위치를 클릭 -> 차 앞 방향으로 드래그\n'
            '  4. 주황 파티클이 한 점으로 뭉치면 성공\n'
            ' (위치를 완전히 잃으면 "Publish Point"로 전체 재분산)\n'
            '=========================================================\n'
        )),
    ] if _is_true(rviz) else [])))

    gate = Node(
        package='integration_pkg',
        executable='wait_for_localization.py',
        name='wait_for_localization',
        output='screen',
        # emulate_tty 없이는 rclpy 로그가 파이프에 물려 버퍼링돼서 화면에도
        # launch.log 에도 안 남는다. 이 게이트는 "왜 안 넘어가는지"를 1초마다
        # 알려주는 게 존재 이유인데, 정작 그 줄이 통째로 사라져 있었다.
        # (2026-07-27 실차 세션: 3분간 아무 메시지 없이 멈춰 있는 것처럼 보였음)
        emulate_tty=True,
        parameters=[{'timeout_sec': float(wait_timeout)}],
    )

    stack_args = {'env': 'real_car'}
    if map_name:
        stack_args['map'] = map_name
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _share('integration_pkg') + '/launch/stack.launch.py'),
        launch_arguments=stack_args.items(),
    )

    def on_gate_exit(event, _context):
        if event.returncode != 0:
            return [LogInfo(msg='localization 수렴 실패 — 자율 스택을 시작하지 '
                                '않습니다. 위 로그를 확인하세요.')]
        return [
            LogInfo(msg='[3/3] 자율 스택 시작 (perception + planning + control)...'),
            stack,
        ]

    # 게이트도 PF가 뜬 뒤에 시작해야 "PF 없음" 로그로 도배되지 않는다.
    actions.append(TimerAction(period=8.0, actions=[gate]))
    if autostart:
        actions.append(RegisterEventHandler(
            OnProcessExit(target_action=gate, on_exit=on_gate_exit)))

    return actions


def generate_launch_description():
    return LaunchDescription([
        # Fast DDS 비동기 발행. 동기(기본) 발행은 RELIABLE /tf writer가 느린
        # 구독자(WiFi 원격 RViz 등)에 막히면 모든 구독자로의 /tf 전달이 밀려
        # perception이 "TF not close enough"로 전멸한다
        # (run_0731_215405 케이스 1, run_0801_175634 재발).
        # ~/.zshrc에도 있지만 launch 밖 환경에 의존하지 않도록 여기서도 박는다.
        SetEnvironmentVariable('RMW_FASTRTPS_PUBLICATION_MODE', 'ASYNCHRONOUS'),
        DeclareLaunchArgument('hardware', default_value='true',
                              description='f1tenth_stack bringup 실행 여부'),
        DeclareLaunchArgument('localization', default_value='true',
                              description='particle_filter localization 실행 여부'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='초기 위치 지정 + 주행 모니터 RViz'
                                          '(monitor_pkg/rviz/monitor.rviz) 실행 여부. '
                                          'false여도 monitor_node는 뜬다'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='수렴 후 자율 스택 자동 실행 여부'),
        DeclareLaunchArgument('wait_timeout', default_value='0.0',
                              description='수렴 대기 제한시간(초). 0이면 무한 대기'),
        DeclareLaunchArgument('map', default_value='',
                              description='맵 이름. PF 맵(particle_filter/maps/<이름>.yaml)과 '
                                          'planner CSV 3종을 한 번에 전환한다. '
                                          '빈 값이면 localize.yaml / params.yaml 기본값 유지'),
        OpaqueFunction(function=launch_setup),
    ])
