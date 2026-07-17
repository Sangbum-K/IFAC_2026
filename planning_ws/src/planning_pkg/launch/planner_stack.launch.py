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
    params_file = os.path.join(
        get_package_share_directory('planning_pkg'), 'config', 'params.yaml')

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

    def make_node(executable):
        params = [params_file]
        if overrides[executable]:
            params.append(overrides[executable])
        return Node(
            package='planning_pkg',
            executable=executable,
            name=executable,
            parameters=params,
            output='screen',
        )

    return [
        make_node('global_trajectory_node'),
        make_node('frenet_frame_node'),
        make_node('local_trajectory_node'),
        make_node('spliner_node'),
        make_node('recovery_supervisor_node'),
        make_node('planner_visualizer_node'),
    ]


def generate_launch_description():
    return LaunchDescription([
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
        OpaqueFunction(function=launch_setup),
    ])
