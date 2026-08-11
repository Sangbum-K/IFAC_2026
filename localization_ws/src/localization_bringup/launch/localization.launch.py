import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _default_map_name():
    """particle_filter/config/localize.yaml 의 `map:` 이름 (맵 이름의 단일 출처)."""
    pf_share = get_package_share_directory('particle_filter')
    with open(os.path.join(pf_share, 'config', 'localize.yaml')) as f:
        return yaml.safe_load(f)['map_server']['ros__parameters']['map']


def _resolve_map(context, *args, **kwargs):
    """map:=<이름> 을 map_yaml_path 로 바꿔 넣는다.

    두 곳을 순서대로 찾는다:
      1) maps_dir (기본 ~/IFAC_ws/maps)  — mapping.launch.py 가 저장하는 곳.
         여기서 먼저 찾기 때문에 새로 만든 맵을 재빌드 없이 바로 쓸 수 있다.
      2) particle_filter/maps (설치본)   — 패키지에 포함돼 배포되는 맵.

    map_yaml_path 를 직접 준 경우엔 건드리지 않는다(전체 경로 override).
    map:= 도 없으면 localize.yaml 의 `map:` 을 따른다 — 기존 동작 그대로.

    여기서 존재 확인을 하는 이유: 맵 파일이 없으면 map_server 가 조용히 죽고
    PF 가 영영 수렴하지 않는다. 원인을 찾기 어려운 실패라 launch 단계에서 막는다.
    """
    if context.launch_configurations.get('map_yaml_path'):
        return []

    map_name = context.launch_configurations.get('map') or _default_map_name()
    search_dirs = [
        os.path.expanduser(context.launch_configurations.get('maps_dir') or ''),
        os.path.join(get_package_share_directory('particle_filter'), 'maps'),
    ]

    for directory in search_dirs:
        path = os.path.join(directory, map_name + '.yaml')
        if directory and os.path.isfile(path):
            context.launch_configurations['map_yaml_path'] = path
            return [LogInfo(msg='map:=%s -> %s' % (map_name, path))]

    listing = []
    for directory in search_dirs:
        if directory and os.path.isdir(directory):
            names = sorted(os.path.splitext(f)[0] for f in os.listdir(directory)
                           if f.endswith('.yaml'))
            listing.append('    %s: %s' % (directory, ', '.join(names) or '(없음)'))
        elif directory:
            listing.append('    %s: (폴더 없음)' % directory)
    raise RuntimeError(
        'map:=%s 의 맵 파일(%s.yaml)을 찾을 수 없습니다.\n'
        '  찾아본 곳:\n%s\n'
        '  (매핑을 먼저 하세요: ros2 launch integration_pkg mapping.launch.py '
        'map_name:=%s)'
        % (map_name, map_name, '\n'.join(listing), map_name))


def generate_launch_description():
    backend = LaunchConfiguration('backend')
    mode = LaunchConfiguration('mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_path = LaunchConfiguration('map_yaml_path')
    posegraph_path = LaunchConfiguration('posegraph_path')
    rviz = LaunchConfiguration('rviz')
    pf_profile = LaunchConfiguration('pf_profile')

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
            'map',
            default_value='',
            description='맵 이름 (<이름>.yaml). maps_dir -> particle_filter/maps 순으로 '
                        '찾는다. 빈 값이면 particle_filter/config/localize.yaml 의 map: 을 따른다.',
        ),
        DeclareLaunchArgument(
            'maps_dir',
            default_value=os.path.expanduser('~/IFAC_ws/maps'),
            description='map:= 을 먼저 찾아볼 폴더 (mapping.launch.py 의 저장 위치)',
        ),
        DeclareLaunchArgument(
            'map_yaml_path',
            default_value='',
            description='맵 yaml 전체 경로 override. 주면 map:= 보다 우선한다. '
                        '비우면 map:= 이름으로 자동 해석.',
        ),
        # map -> map_yaml_path 해석은 아래 노드들이 방문되기 전에 끝나야 한다.
        OpaqueFunction(function=_resolve_map),
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
        DeclareLaunchArgument(
            'pf_profile',
            default_value='variant0_no_ground_no_deskew',
            description='C++ particle_filter 프로파일.',
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
                'pf_profile': pf_profile,
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
