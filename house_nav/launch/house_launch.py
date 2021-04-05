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
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    package_dir = get_package_share_directory('house_nav')
      
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    pick_cmd = Node(
        package='house_nav',
        executable='pick_action_node',
        name='pick_action_node',
        
        output='screen',
        parameters=[])

    place_cmd = Node(
        package='house_nav',
        executable='place_action_node',
        name='place_action_node',
        
        output='screen',
        parameters=[])

    move_between_rooms_cmd = Node(
    package='house_nav',
    executable='move_between_rooms_action_node',
    output='screen',
    parameters=[os.path.join(package_dir, 'config','params.yaml')])

    enter_zone_cmd = Node(
    package='house_nav',
    executable='enter_zone_action_node',
    output='screen',
    parameters=[os.path.join(package_dir, 'config','params.yaml')])

    leave_zone_cmd = Node(
    package='house_nav',
    executable='leave_zone_action_node',
    output='screen',
    parameters=[os.path.join(package_dir, 'config','params.yaml')])

    move_between_zones_cmd = Node(
    package='house_nav',
    executable='move_between_zones_action_node',
    output='screen',
    parameters=[os.path.join(package_dir, 'config','params.yaml')])

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(pick_cmd)
    ld.add_action(place_cmd)
    ld.add_action(move_between_rooms_cmd)
    ld.add_action(enter_zone_cmd)
    ld.add_action(leave_zone_cmd)
    ld.add_action(move_between_zones_cmd)


    return ld