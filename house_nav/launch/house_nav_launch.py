# Copyright 2021 The Rebooters
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    package_dir = get_package_share_directory('house_nav')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robots_dir = get_package_share_directory('robots')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
          #'model_file': package_dir + '/pddl/domain.pddl',
          'model_file': '/home/noeljg/Desktop/cognitive-systems-projects/pddl/star_trips_world/star_trips2_domain.pddl',
          'namespace': namespace
          }.items())

    webots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('webots_ros2_core'), 'launch', 'robot_launch.py')
        ),
        launch_arguments={
            'executable': 'webots_differential_drive_node',
            'world': os.path.join(robots_dir, 'worlds', 'complete_apartment.wbt'),
            'node_parameters': os.path.join(robots_dir, 'resource', 'tiago.yaml'),
            'use_sim_time': use_sim_time
        }.items()
    )


    # Create the launch description
    ld = LaunchDescription()
    
    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # Declare the launch options
    ld.add_action(plansys2_cmd)
    ld.add_action(webots)

    return ld
