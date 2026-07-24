"""전체 스택 통합 launch (perception + planning + control + integration 모니터).

  # gym 통합 시뮬 (f1tenth_gym_ros + 실제 perception 체인 + C++ control_node):
  ros2 launch integration_pkg stack.launch.py                 # env:=sim 기본
  ros2 launch integration_pkg stack.launch.py map:=Budapest_map rviz:=false

  # 실차 (hardware/localization은 각 팀 launch로 별도 기동):
  ros2 launch integration_pkg stack.launch.py env:=real_car

env:=sim 데이터 흐름 (실차와 동일 토폴로지를 gym 위에 재현):
  gym /scan -> virtual_obstacle(lidar_overlay, 클릭 장애물 합성) -> /scan_overlay
    -> perception_pkg obstacle_detector -> /perception/obstacles_detected (laser frame)
    -> planning perception_obstacle_bridge -> /perception/obstacles_frenet
    -> spliner/recovery.  control_ws control_node가 /local_waypoints -> /drive.

env:=real_car 전제:
  perception 팀 노드가 /perception/obstacles(f1tenth_perception_msgs) 발행,
  localization_ws particle_filter가 /pf/pose/odom + map->odom TF 발행,
  hardware_ws가 /scan·/odom·estop 제공. 여기서는 perception/planning/control/
  integration만 기동한다.

전제 source 순서: ros humble -> (sim이면 ~/sim_ws) -> planning_ws -> perception_ws
  -> control_ws -> integration_ws.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    env = LaunchConfiguration('env').perform(context)
    planning_share = get_package_share_directory('planning_pkg')
    perception_share = get_package_share_directory('perception_pkg')
    control_share = get_package_share_directory('control_pkg')
    integration_share = get_package_share_directory('integration_pkg')

    actions = []

    map_name = LaunchConfiguration('map').perform(context)

    if env == 'sim':
        # gym + planner 스택 + sim 보조 노드 + RViz.
        # tracker:=false — python tracker 대신 control_ws C++ control_node 사용.
        # perception:=lidar — GT 주입 대신 실제 perception 체인.
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(planning_share, 'launch', 'sim_race.launch.py')),
            launch_arguments={
                # sim은 gym 맵 이름이 따로 있어 기본값을 여기서 채운다.
                'map': map_name or 'Spielberg_map',
                'rviz': LaunchConfiguration('rviz').perform(context),
                'tracker': 'false',
                'perception': 'lidar',
            }.items(),
        ))
    else:
        # 실차/race_map: map:=<이름>이 주어졌을 때만 CSV 3종을 그 맵으로 바꾼다.
        # 빈 값이면 params.yaml 값을 그대로 둔다 (기존 동작).
        planner_args = {'env': env}
        if map_name:
            planner_args['map'] = map_name
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(planning_share, 'launch', 'planner_stack.launch.py')),
            launch_arguments=planner_args.items(),
        ))

    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perception_share, 'launch', 'perception.launch.py')),
        launch_arguments={'env': env}.items(),
    ))
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(control_share, 'launch', 'control.launch.py')),
        launch_arguments={'env': env}.items(),
    ))
    actions.append(Node(
        package='integration_pkg',
        executable='integration_node',
        name='integration',
        output='screen',
        parameters=[
            os.path.join(integration_share, 'config', 'params.yaml'),
            os.path.join(integration_share, 'config', 'environments', env + '.yaml'),
        ],
    ))
    return actions



def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('env', default_value='sim',
                              description='sim | real_car | race_map | mapping'),
        DeclareLaunchArgument('map', default_value='',
                              description='맵 이름. env:=sim이면 gym 맵(기본 '
                                          'Spielberg_map), 그 외에는 planner의 '
                                          'CSV 3종을 이 맵으로 전환. 빈 값이면 '
                                          'params.yaml 값 유지'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='(env:=sim) RViz 실행 여부'),
        OpaqueFunction(function=launch_setup),
    ])
