"""raceline 추종 파이프라인 launch.

global_trajectory_node -> frenet_frame_node -> local_trajectory_node
                                            -> spliner_node (장애물 회피)
                                            -> planner_visualizer_node

파라미터는 config/params.yaml에서 로드하고, 자주 바꾸는 것만 CLI 인자로
override 가능:

  ros2 launch planning_pkg planner_stack.launch.py
  ros2 launch planning_pkg planner_stack.launch.py odom_mode:=real
  ros2 launch planning_pkg planner_stack.launch.py \
      trajectory_csv_file:=<다른 raceline>.csv centerline_csv_file:=<맵>_centerline.csv

(기존 planner_pkg_launch.py를 대체: 존재하지 않는 planner_config.yaml 참조와
trajectory 파일명에서 centerline 이름을 소수점 파싱으로 유도하던 버그를 제거하고,
centerline은 명시적 인자/params.yaml 기본값으로만 정한다.)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
        'planner_visualizer_node': {},
    }
    traj_csv = LaunchConfiguration('trajectory_csv_file').perform(context)
    if traj_csv:
        overrides['global_trajectory_node'][
            'global_trajectory/trajectory_csv_file'] = traj_csv
    ot_csv = LaunchConfiguration('overtaking_csv_file').perform(context)
    if ot_csv:
        overrides['global_trajectory_node'][
            'global_trajectory/overtaking_csv_file'] = ot_csv
    odom_mode = LaunchConfiguration('odom_mode').perform(context)
    if odom_mode:
        overrides['frenet_frame_node']['frenet_frame/odom_mode'] = odom_mode
    pose_topic = LaunchConfiguration('pose_odom_topic').perform(context)
    if pose_topic:
        overrides['recovery_supervisor_node'][
            'recovery/pose_odom_topic'] = pose_topic
    center_csv = LaunchConfiguration('centerline_csv_file').perform(context)
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

    nodes = [
        make_node('global_trajectory_node'),
        make_node('frenet_frame_node'),
        make_node('local_trajectory_node'),
        make_node('spliner_node'),
        make_node('recovery_supervisor_node'),
        make_node('planner_visualizer_node'),
    ]

    # 실차 인터페이스 브릿지 (sim은 virtual_obstacle/gym GT가 대체):
    #  - perception 팀 laser-frame 장애물 -> frenet (/perception/obstacles_frenet)
    # /pf/pose/odom은 localization_ws particle_filter가 직접 발행 (publish_odom: 1)
    # — 플래닝 쪽에서 별도 pose 브릿지를 띄우지 않는다 (이중 발행 금지).
    bridge = LaunchConfiguration('bridge').perform(context).lower() == 'true'
    if env in ('real_car', 'race_map') or bridge:
        bridge_input = LaunchConfiguration('bridge_input_topic').perform(context)
        if bridge_input:
            overrides['perception_obstacle_bridge_node.py'] = {
                'bridge/input_topic': bridge_input}
        nodes.append(make_node('perception_obstacle_bridge_node.py'))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('env', default_value='',
                              description='환경 프로파일 (config/environments/<env>.yaml): '
                                          '빈 값=sim 기본, real_car | race_map | mapping'),
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
