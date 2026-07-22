import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _default_map_yaml_path():
    """particle_filter/config/localize.yaml 의 `map:` 이름을 실제 yaml 경로로.

    이 값은 아래에서 localize_launch.py 로 그대로 넘어간다. 예전 기본값은
    존재하지 않는 /tmp/f1tenth_track_map.yaml 이었고, 그게 localize_launch.py
    자신의 기본값(패키지 maps/<map>.yaml)을 덮어써서 map_server 가 맵을 못 읽고
    조용히 죽었다 — PF 가 영영 수렴하지 않는 원인이었다.
    """
    pf_share = get_package_share_directory('particle_filter')
    with open(os.path.join(pf_share, 'config', 'localize.yaml')) as f:
        map_name = yaml.safe_load(f)['map_server']['ros__parameters']['map']
    return os.path.join(pf_share, 'maps', map_name + '.yaml')


def generate_launch_description():
    backend = LaunchConfiguration('backend')
    mode = LaunchConfiguration('mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_path = LaunchConfiguration('map_yaml_path')
    posegraph_path = LaunchConfiguration('posegraph_path')
    rviz = LaunchConfiguration('rviz')

    pf_rviz_config = PathJoinSubstitution([
        FindPackageShare('particle_filter'),
        'rviz',
        'pf.rviz',
    ])

    slam_mapping_config = PathJoinSubstitution([
        FindPackageShare('localization_bringup'),
        'config',
        'localization',
        'slam_toolbox_mapping.yaml',
    ])
    slam_localization_config = PathJoinSubstitution([
        FindPackageShare('localization_bringup'),
        'config',
        'localization',
        'slam_toolbox_localization.yaml',
    ])
    amcl_config = PathJoinSubstitution([
        FindPackageShare('localization_bringup'),
        'config',
        'localization',
        'amcl.yaml',
    ])
    map_server_config = PathJoinSubstitution([
        FindPackageShare('localization_bringup'),
        'config',
        'localization',
        'map_server.yaml',
    ])

    use_amcl = IfCondition(PythonExpression(["'", backend, "' == 'amcl'"]))
    use_particle_filter = IfCondition(PythonExpression([
        "'", backend, "' == 'particle_filter' and '", mode, "' == 'localization'"
    ]))
    # 파티클 필터는 켜자마자 맵 전체에 파티클을 흩뿌린 상태라 사람이 RViz에서
    # 2D Pose Estimate로 초기 위치를 찍어줘야 한다. 그래서 RViz를 같이 띄운다.
    show_pf_rviz = IfCondition(PythonExpression([
        "'", backend, "' == 'particle_filter' and '", mode, "' == 'localization'"
        " and '", rviz, "' in ('true', 'True', '1')"
    ]))

    return LaunchDescription([
        DeclareLaunchArgument(
            'backend',
            default_value='slam_toolbox',
            description='Localization backend: slam_toolbox, amcl, or particle_filter',
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='localization',
            description='Localization mode: mapping or localization. particle_filter supports localization only.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true.',
        ),
        DeclareLaunchArgument(
            'map_yaml_path',
            default_value=_default_map_yaml_path(),
            description='Map yaml path for amcl or particle_filter localization. '
                        '기본값은 particle_filter/config/localize.yaml 의 map: 이름을 따른다.',
        ),
        DeclareLaunchArgument(
            'posegraph_path',
            default_value='/tmp/f1tenth_track_posegraph',
            description='Slam toolbox posegraph path for localization mode.',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='(backend:=particle_filter) 초기 위치 지정용 RViz 실행 여부',
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_mapping_config, {'use_sim_time': use_sim_time}],
            condition=IfCondition(PythonExpression([
                "'", backend, "' == 'slam_toolbox' and '", mode, "' == 'mapping'"
            ])),
        ),
        Node(
            package='slam_toolbox',
            executable='localization_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_localization_config,
                {
                    'use_sim_time': use_sim_time,
                    'map_file_name': posegraph_path,
                },
            ],
            condition=IfCondition(PythonExpression([
                "'", backend, "' == 'slam_toolbox' and '", mode, "' == 'localization'"
            ])),
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                map_server_config,
                {
                    'use_sim_time': use_sim_time,
                    'yaml_filename': map_yaml_path,
                },
            ],
            condition=use_amcl,
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_config, {'use_sim_time': use_sim_time}],
            condition=use_amcl,
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server', 'amcl'],
            }],
            condition=use_amcl,
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('particle_filter'),
                    'launch',
                    'localize_launch.py',
                ])
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map_yaml_path': map_yaml_path,
            }.items(),
            condition=use_particle_filter,
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_localization',
            output='log',
            arguments=['-d', pf_rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=show_pf_rviz,
        ),
        LogInfo(
            msg=(
                '\n'
                '=========================================================\n'
                ' 파티클 필터 초기 위치를 지정하세요 (RViz 창):\n'
                '  1. 차를 맵상 아는 지점(보통 출발선)에 실제로 갖다 놓기\n'
                '  2. 상단 툴바 "2D Pose Estimate" 클릭\n'
                '  3. 맵에서 차가 있는 위치를 클릭 → 차 앞 방향으로 드래그\n'
                '  4. 주황 파티클이 한 점으로 뭉치면 성공\n'
                ' (위치를 완전히 잃으면 "Publish Point"로 전체 재분산)\n'
                '=========================================================\n'
            ),
            condition=show_pf_rviz,
        ),
    ])
