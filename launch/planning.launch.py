"""planning 단독 실행: planning_pkg planner_stack (env 프로파일 적용).

sim 전체(gym 포함)는 integration.launch.py 또는
`ros2 launch planning_pkg sim_race.launch.py`를 사용.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('env', default_value='',
                              description='빈 값=sim 기본 | real_car | race_map | mapping'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('planning_pkg'),
                'launch', 'planner_stack.launch.py')),
            launch_arguments={'env': LaunchConfiguration('env')}.items(),
        ),
    ])
