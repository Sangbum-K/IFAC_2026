from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    backend = LaunchConfiguration('backend')
    mode = LaunchConfiguration('mode')

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

    return LaunchDescription([
        DeclareLaunchArgument(
            'backend',
            default_value='slam_toolbox',
            description='Localization backend: slam_toolbox or amcl',
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='localization',
            description='Localization mode: mapping or localization',
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_mapping_config],
            condition=IfCondition(PythonExpression([
                "'", backend, "' == 'slam_toolbox' and '", mode, "' == 'mapping'"
            ])),
        ),
        Node(
            package='slam_toolbox',
            executable='localization_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_localization_config],
            condition=IfCondition(PythonExpression([
                "'", backend, "' == 'slam_toolbox' and '", mode, "' == 'localization'"
            ])),
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[map_server_config],
            condition=use_amcl,
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_config],
            condition=use_amcl,
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['map_server', 'amcl'],
            }],
            condition=use_amcl,
        ),
    ])
