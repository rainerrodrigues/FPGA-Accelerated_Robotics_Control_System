#!/usr/bin/env python3
# spawn_robot.launch.py
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = os.path.dirname(__file__)  # not strictly necessary
    urdf_path = os.path.expanduser(os.path.join(os.getcwd(), "robot", "robot.urdf"))
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', urdf_path, '-entity', 'rsk_robot'],
        output='screen'
    )

    # Launch gazebo (if you want it started by this launch)
    gz = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    return LaunchDescription([
        gz,
        spawn
    ])
