# Copyright 2020 The Reboteers

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():

    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    practica_1_cmd = Node(
        package='scan_fake',
        node_executable='practica_1',
        output='screen',
        parameters=[])

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(practica_1_cmd)

    return ld
