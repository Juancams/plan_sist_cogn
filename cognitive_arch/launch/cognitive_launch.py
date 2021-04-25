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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    package_dir = get_package_share_directory('cognitive_arch')
    house_dir = get_package_share_directory('house_nav')
    bringup_dir = get_package_share_directory('nav2_bringup')
    namespace = LaunchConfiguration('namespace')
    robots_dir = get_package_share_directory('robots')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(package_dir, 'rviz', 'cognitive_arch_config.rviz'),
        description='Full path to the RVIZ config file to use')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
          'model_file':
          package_dir + '/pddl/cognitive_arch_domain.pddl',
          'namespace': namespace
        }.items()
    )

    webots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
          os.path.join(os.path.join(robots_dir, 'launch', 'tiago_nodoors.launch.py'))
        ),
        launch_arguments={
          'namespace': namespace
        }.items()
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
          bringup_dir,
          'launch',
          'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={
          'namespace': '',
          'use_namespace': 'False',
          'rviz_config': rviz_config_file
        }.items()
    )

    # Specify the actions
    move_cmd = Node(
        package='cognitive_arch',
        executable='move_action',
        output='screen',
        parameters=[os.path.join(house_dir, 'config', 'params.yaml')])

    explore_cmd = Node(
        package='cognitive_arch',
        executable='explore_action',
        name='explore_action',
        output='screen',
        parameters=[])

    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)

    # Declare the launch options
    ld.add_action(plansys2_cmd)
    ld.add_action(webots)
    ld.add_action(rviz_cmd)

    ld.add_action(move_cmd)
    ld.add_action(explore_cmd)

    return ld
