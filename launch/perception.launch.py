"""perception 단독 실행: perception_pkg obstacle_detector (env 프로파일 적용)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('env', default_value='sim',
                              description='sim | real_car | race_map | mapping'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('perception_pkg'),
                'launch', 'perception.launch.py')),
            launch_arguments={'env': LaunchConfiguration('env')}.items(),
        ),
    ])
