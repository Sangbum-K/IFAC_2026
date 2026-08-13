"""맵 작성 원커맨드 launch — 하드웨어 + slam_toolbox + 맵 저장을 한 번에.

  ros2 launch integration_pkg mapping.launch.py map_name:=내맵이름

이 한 줄이 아래를 띄운다:
  A) f1tenth_stack bringup_launch.py    (LiDAR, VESC, joy, mux, TF)
  B) slam_toolbox (async, mapping 모드)  — 5초 뒤 기동 (/scan·TF가 먼저 나오게)
  C) RViz                                — 맵이 그려지는 걸 보면서 주행
  D) map_saver_node                      — /map을 캐시했다가 파일로 저장

사람이 할 일: **조이스틱으로 트랙을 한 바퀴 천천히 돌고 Ctrl+C.**
Ctrl+R은 2초 안에 두 번 누르면 현재 pose graph를 버리고 현재 위치부터
다시 매핑한다. Ctrl+C는 저장 후 종료한다.

저장 위치는 기본이 **~/IFAC_ws/maps/** 다 (워크스페이스 공용 맵 폴더).

주요 인자:
  map_name:=track1      저장할 맵 이름 (기본 my_map)
  map_dir:=<경로>       저장 디렉터리 (기본 ~/IFAC_ws/maps)
  hardware:=false       이미 A가 떠 있을 때
  rviz:=false           화면 없이
  save_on_exit:=false   Ctrl+C 자동 저장을 끄고 /save_map 서비스로만 저장
  keyboard_controls:=false  Ctrl+R 터미널 단축키 비활성화
  overwrite:=false      이름이 이미 쓰일 때만 거부. 기본은 덮어쓰기 허용

이름이 겹치면 **주행 전에** 막는다: map_dir 은 localization / raceline_tuner /
build_map / sim_race 가 모두 먼저 뒤지는 폴더라, 기존 맵이나 패키지 맵
(particle_filter_cpp/maps, planning_pkg/maps)과 같은 이름으로 저장하면 그 맵이
에러 없이 조용히 덮어쓰이거나 가려진다.

주행 중 중간 저장:
  ros2 service call /save_map std_srvs/srv/Trigger

터미널이 없을 때 처음부터 다시:
  ros2 service call /reset_map std_srvs/srv/Trigger

저장 후 그 맵으로 주행하려면 — **재빌드 불필요**:
  ros2 launch integration_pkg real_car.launch.py map:=track1
  (localization.launch.py 의 map:= 이 ~/IFAC_ws/maps 를 먼저 찾는다.
   단 planner raceline CSV는 별도로 scripts/global_traj_sim/build_map.py 로 뽑아야 한다)

전제 source: ros humble -> integration_ws -> hardware_ws -> localization_ws
"""

import os
import select
import termios
import threading
import time

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node as RclpyNode
from rclpy.signals import SignalHandlerOptions
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from std_srvs.srv import Trigger

# 워크스페이스 공용 맵 폴더. localization.launch.py 의 map:= 가 이 폴더를
# 먼저 뒤지므로, 여기에 저장하면 재빌드 없이 바로 map:=<이름> 으로 주행할 수 있다.
DEFAULT_MAP_DIR = os.path.expanduser('~/IFAC_ws/maps')
CTRL_R = b'\x12'
RESET_CONFIRM_SEC = 2.0


class MappingKeyboardController:
    """Read mapping hotkeys in the launch process and call saver services.

    ROS launch children receive a pipe as stdin, so the child map_saver_node
    cannot portably read the user's terminal.  The launch process owns that
    terminal and forwards key intent through ROS services instead.
    """

    def __init__(self):
        self.stop_event = threading.Event()
        self.stop_lock = threading.Lock()
        self.stopped = False
        self.key_thread = None
        self.spin_thread = None
        self.ros_context = None
        self.node = None
        self.executor = None
        self.reset_client = None
        self.reset_armed_until = 0.0

    @staticmethod
    def report(message):
        print('[mapping-keys] %s' % message, flush=True)

    def start(self, _context, *args, **kwargs):
        try:
            self.ros_context = Context()
            rclpy.init(
                args=[], context=self.ros_context,
                signal_handler_options=SignalHandlerOptions.NO)
            self.node = RclpyNode(
                'mapping_keyboard_controller', context=self.ros_context)
            self.reset_client = self.node.create_client(Trigger, '/reset_map')
            self.executor = SingleThreadedExecutor(context=self.ros_context)
            self.executor.add_node(self.node)
            self.spin_thread = threading.Thread(
                target=self.executor.spin, name='mapping-keyboard-ros',
                daemon=True)
            self.spin_thread.start()
            self.key_thread = threading.Thread(
                target=self.keyboard_loop, name='mapping-keyboard-input',
                daemon=True)
            self.key_thread.start()
            self.report(
                'Ctrl+R 두 번=처음부터 다시 | Ctrl+C=저장 후 종료')
        except Exception as error:
            self.report('키보드 제어 시작 실패: %s' % error)
            self.stop(_context)
        return []

    def call_trigger(self, client, label):
        if self.stop_event.is_set():
            return
        if client is None or not client.service_is_ready():
            self.report('%s 서비스가 아직 준비되지 않았습니다.' % label)
            return
        future = client.call_async(Trigger.Request())

        def done(completed):
            try:
                response = completed.result()
                state = '성공' if response.success else '실패'
                self.report('%s %s: %s' % (label, state, response.message))
            except Exception as error:
                self.report('%s 호출 실패: %s' % (label, error))

        future.add_done_callback(done)

    def keyboard_loop(self):
        fd = None
        old_attrs = None
        try:
            try:
                fd = os.open('/dev/tty', os.O_RDONLY | os.O_NONBLOCK)
            except OSError:
                if not os.isatty(0):
                    raise
                fd = os.dup(0)
                os.set_blocking(fd, False)
            old_attrs = termios.tcgetattr(fd)
            attrs = list(old_attrs)
            attrs[6] = list(old_attrs[6])
            # 터미널 흐름 제어 키를 실수로 눌러 출력이 멈추는 것도 방지한다.
            # Ctrl+R은 Enter 없이 한 글자씩 읽는다.
            attrs[0] &= ~(termios.IXON | termios.IXOFF)
            if hasattr(termios, 'IXANY'):
                attrs[0] &= ~termios.IXANY
            attrs[3] &= ~(termios.ICANON | termios.ECHO)
            # ISIG는 유지하므로 Ctrl+C는 launch의 정상 종료 신호로 남는다.
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 1
            termios.tcsetattr(fd, termios.TCSANOW, attrs)

            while not self.stop_event.is_set():
                readable, _, _ = select.select([fd], [], [], 0.10)
                if not readable:
                    continue
                key = os.read(fd, 1)
                if key == CTRL_R:
                    now = time.monotonic()
                    if now > self.reset_armed_until:
                        self.reset_armed_until = now + RESET_CONFIRM_SEC
                        self.report(
                            '[Ctrl+R] 차량을 멈추고, 초기화하려면 2초 안에 '
                            'Ctrl+R을 한 번 더 누르세요.')
                    else:
                        self.reset_armed_until = 0.0
                        self.report('[Ctrl+R] 처음부터 다시 매핑 요청')
                        self.call_trigger(self.reset_client, '/reset_map')
        except (OSError, termios.error) as error:
            self.report(
                '키보드 제어 비활성(터미널 없음: %s). /save_map 및 '
                '/reset_map 서비스를 사용하세요.' % error)
        finally:
            if fd is not None and old_attrs is not None:
                try:
                    termios.tcsetattr(fd, termios.TCSANOW, old_attrs)
                except (OSError, termios.error):
                    pass
            if fd is not None:
                os.close(fd)

    def stop(self, _context=None, *args, **kwargs):
        # start() 실패 뒤에도 OnShutdown이 호출될 수 있으므로 정리를 멱등하게 한다.
        with self.stop_lock:
            if self.stopped:
                return []
            self.stopped = True
            self.stop_event.set()
            if self.key_thread is not None:
                self.key_thread.join(timeout=0.5)
            if self.executor is not None:
                self.executor.shutdown(timeout_sec=0.5)
            if self.spin_thread is not None:
                self.spin_thread.join(timeout=0.5)
            if self.node is not None:
                self.node.destroy_node()
                self.node = None
            if self.ros_context is not None and self.ros_context.ok():
                rclpy.shutdown(context=self.ros_context)
        return []


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
    ('particle_filter_cpp', 'maps'),  # localization 이 2순위로 찾는 곳
    # build_map / sim_race 는 map_dir 한 곳만 보게 바뀌었지만, raceline_tuner 는
    # 아직 아니다 — tuner/line_resolver.py 의 MAPS_SEARCH_DIRS 가 여기를 2순위로
    # 뒤지므로 여전히 가려질 수 있다.
    ('planning_pkg', 'maps'),
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
    keyboard_controls = _is_true(
        LaunchConfiguration('keyboard_controls').perform(context))
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
                    FindPackageShare('particle_filter_cpp'), 'rviz', 'pf.rviz'])],
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

    if keyboard_controls:
        keyboard = MappingKeyboardController()
        actions.append(RegisterEventHandler(OnShutdown(
            on_shutdown=[OpaqueFunction(function=keyboard.stop)])))
        actions.append(OpaqueFunction(function=keyboard.start))

    actions.append(LogInfo(msg=(
        '\n'
        '=========================================================\n'
        ' [3/3] 맵 작성 준비 완료\n'
        '   1. 조이스틱으로 트랙을 천천히 한 바퀴 도세요\n'
        '      (RViz에서 맵이 채워지는 걸 확인하며 — 급회전은 정합 실패 유발)\n'
        '   2. 잘못 그렸으면 Ctrl+R 두 번 (현재 위치부터 처음부터 다시)\n'
        '   3. 출발선으로 돌아와 루프가 닫히면 Ctrl+C (저장 후 종료)\n'
        '   4. 저장 위치: %s\n'
        '   (서비스: /save_map=저장, /reset_map=처음부터 다시)\n'
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
        DeclareLaunchArgument('keyboard_controls', default_value='true',
                              description='Ctrl+R 두 번 초기화 단축키'),
        DeclareLaunchArgument('overwrite', default_value='true',
                              description='기존 맵/패키지 맵과 이름이 겹쳐도 진행 '
                                          '(기본). false면 launch 단계에서 거부. '
                                          '겹친 파일은 저장 시 .bak 으로 백업된다'),
        OpaqueFunction(function=launch_setup),
    ])
