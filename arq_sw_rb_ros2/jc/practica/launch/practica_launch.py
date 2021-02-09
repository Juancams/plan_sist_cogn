# Copyright 2020 Juan Carlos Manzanares Serrano
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

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():

    stdout_linebuf_envvar = SetEnvironmentVariable(
      'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    proceso_1_cmd = Node(
      package='practica',
      node_executable='proceso_1',
      output='screen',
      parameters=[])

    proceso_2_cmd = Node(
      package='practica',
      node_executable='proceso_2',
      output='screen',
      parameters=[])

    proceso_3_cmd = Node(
      package='practica',
      node_executable='proceso_3',
      output='screen',
      parameters=[])

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(proceso_1_cmd)
    ld.add_action(proceso_2_cmd)
    ld.add_action(proceso_3_cmd)

    return ld
