from launch import LaunchDescription
from launch_ros.actions import Node

#!/usr/bin/env python3


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='leap_hand_control',
            executable='ring_manager',
            name='ring_node',
            output='screen'
        ),
        Node(
            package='leap_hand_control',
            executable='middle_manager',
            name='middle_node',
            output='screen'
        ),
        Node(
            package='leap_hand_control',
            executable='index_manager',
            name='index_node',
            output='screen'
        ),
        Node(
            package='leap_hand_control',
            executable='thumb_manager',
            name='thumb_node',
            output='screen'
        ),
    ])
