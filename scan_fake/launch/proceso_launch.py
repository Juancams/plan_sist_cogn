# Copyright 2021 The Rebooters

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():

    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    proceso_cmd = Node(
        package='scan_fake',
        node_executable='proceso',
        output='screen',
        parameters=[])

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(proceso_cmd)

    return ld
