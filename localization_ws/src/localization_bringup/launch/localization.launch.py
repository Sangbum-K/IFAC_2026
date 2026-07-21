from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    backend = LaunchConfiguration('backend')
    mode = LaunchConfiguration('mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_path = LaunchConfiguration('map_yaml_path')
    posegraph_path = LaunchConfiguration('posegraph_path')

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
            default_value='/tmp/f1tenth_track_map.yaml',
            description='Map yaml path for amcl or particle_filter localization.',
        ),
        DeclareLaunchArgument(
            'posegraph_path',
            default_value='/tmp/f1tenth_track_posegraph',
            description='Slam toolbox posegraph path for localization mode.',
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
    ])
