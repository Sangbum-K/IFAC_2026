# Copyright 2026 IFAC F1TENTH Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# The original launch file remains available under its MIT license:
# Copyright 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


PF_AB_PROFILES = (
    'variant0_no_ground_no_deskew',
    'variant0_no_ground_no_deskew_no_icp',
)


def _launch_particle_filter(context, *args, **kwargs):
    """Validate and layer one named A/B profile over the base PF config."""
    profile = LaunchConfiguration('pf_profile').perform(context).strip()
    if profile not in PF_AB_PROFILES:
        raise RuntimeError(
            'Unknown pf_profile:=%r. Choose one of: %s'
            % (profile, ', '.join(PF_AB_PROFILES)))
    base_config = LaunchConfiguration('localize_config').perform(context)
    if not os.path.isfile(base_config):
        raise RuntimeError(
            'PF base config does not exist: %s' % base_config)

    profile_config = os.path.join(
        get_package_share_directory('particle_filter'),
        'config',
        'profiles',
        profile + '.yaml',
    )
    if not os.path.isfile(profile_config):
        raise RuntimeError(
            'PF A/B profile is not installed: %s. Rebuild and source '
            'localization_ws.' % profile_config)

    return [
        LogInfo(msg=(
            '[PF C++] profile=%s; base=%s; overlay=%s'
            % (profile, base_config, profile_config))),
        Node(
            package='particle_filter_cpp',
            executable='particle_filter_cpp_node',
            name='particle_filter',
            output='screen',
            # ROS parameter files are applied left-to-right. The small named
            # profile therefore changes only the feature under test while the
            # production localize.yaml remains the common baseline.
            parameters=[
                base_config,
                profile_config,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
    ]


def generate_launch_description():
    # config and args
    localize_config = os.path.join(
        get_package_share_directory('particle_filter'),
        'config',
        'localize.yaml'
    )
    localize_config_dict = yaml.safe_load(open(localize_config, 'r'))
    map_name = localize_config_dict['map_server']['ros__parameters']['map']
    localize_la = DeclareLaunchArgument(
        'localize_config',
        default_value=localize_config,
        description='Localization configs')
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true.')
    map_yaml_path_la = DeclareLaunchArgument(
        'map_yaml_path',
        default_value=os.path.join(
            get_package_share_directory('particle_filter'),
            'maps',
            map_name + '.yaml',
        ),
        description='Map yaml path used by the PF map_server.',
    )
    pf_profile_la = DeclareLaunchArgument(
        'pf_profile',
        default_value='variant0_no_ground_no_deskew',
        description=(
            'C++ PF profile: %s.'
            % ', '.join(PF_AB_PROFILES)),
    )
    ld = LaunchDescription([
        localize_la,
        use_sim_time_la,
        map_yaml_path_la,
        pf_profile_la,
    ])

    # nodes
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': LaunchConfiguration('map_yaml_path')},
                    {'topic': 'map'},
                    {'frame_id': 'map'},
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )

    # finalize
    ld.add_action(nav_lifecycle_node)
    ld.add_action(map_server_node)
    ld.add_action(OpaqueFunction(function=_launch_particle_filter))

    return ld
