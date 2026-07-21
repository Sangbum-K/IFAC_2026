"""실차 원커맨드 launch — 하드웨어 → localization(+RViz) → 자율 스택을 자동 순서로.

  ros2 launch integration_pkg real_car.launch.py

기존에 터미널 3개로 나눠 하던 A/B/C를 하나로 합친 것이다. 사람이 할 일은
RViz에서 "2D Pose Estimate"로 초기 위치를 한 번 찍는 것뿐이고, 파티클이
수렴하면 자율 스택이 알아서 뜬다.

  A) f1tenth_stack bringup_launch.py       (LiDAR, VESC, joy, mux, TF)
  B) localization_bringup localization.launch.py backend:=particle_filter
     + RViz(pf.rviz)                        — 5초 뒤 기동 (/scan·/odom이 먼저 나오게)
  G) wait_for_localization                  — 파티클 수렴까지 대기하는 게이트
  C) integration_pkg stack.launch.py env:=real_car   — G가 통과하면 자동 실행

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
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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

    actions = []

    if hardware:
        actions.append(LogInfo(msg='[1/3] 하드웨어 bringup (LiDAR/VESC/joy/mux)...'))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _share('f1tenth_stack') + '/launch/bringup_launch.py'),
        ))

    if localization:
        # /scan·/odom이 먼저 흐르게 잠깐 두고 PF를 띄운다. PF는 시작 직후
        # laser->base_link TF를 조회하는데, 그게 없으면 초기 포즈 클릭이 무시된다.
        actions.append(TimerAction(period=5.0, actions=[
            LogInfo(msg='[2/3] localization (particle_filter) + RViz...'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _share('localization_bringup') + '/launch/localization.launch.py'),
                launch_arguments={
                    'backend': 'particle_filter',
                    'mode': 'localization',
                    'use_sim_time': 'false',
                    'rviz': rviz,
                }.items(),
            ),
        ]))

    gate = Node(
        package='integration_pkg',
        executable='wait_for_localization.py',
        name='wait_for_localization',
        output='screen',
        parameters=[{'timeout_sec': float(wait_timeout)}],
    )

    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _share('integration_pkg') + '/launch/stack.launch.py'),
        launch_arguments={'env': 'real_car'}.items(),
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
        DeclareLaunchArgument('hardware', default_value='true',
                              description='f1tenth_stack bringup 실행 여부'),
        DeclareLaunchArgument('localization', default_value='true',
                              description='particle_filter localization 실행 여부'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='초기 위치 지정용 RViz 실행 여부'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='수렴 후 자율 스택 자동 실행 여부'),
        DeclareLaunchArgument('wait_timeout', default_value='0.0',
                              description='수렴 대기 제한시간(초). 0이면 무한 대기'),
        OpaqueFunction(function=launch_setup),
    ])
