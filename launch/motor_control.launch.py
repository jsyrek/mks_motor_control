#!/usr/bin/env python3
"""
Launch motor control nodes:
- Table initializer (sets robot position)
- Hybrid localization (odometry + LiDAR)
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mks_motor_control',
            executable='initialize_robot_on_table',
            name='table_initializer',
            output='screen',
        ),
        Node(
            package='mks_motor_control',
            executable='hybrid_localization',
            name='hybrid_localization',
            output='screen',
        ),
    ])
