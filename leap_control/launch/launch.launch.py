from launch import LaunchDescription
from launch.actions import ExecuteProcess

#!/usr/bin/env python3

def generate_launch_description():
    return LaunchDescription([
        # Terminal 1 - Managers
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'bash', '-c',
                 'ros2 run leap_hand_control hand_manager & '
                 'ros2 run leap_hand_control thumb_manager & '
                 'ros2 run leap_hand_control index_manager & '
                 'ros2 run leap_hand_control ring_manager & '
                 'ros2 run leap_hand_control middle_manager; exec bash'],
            shell=False
        ),

        # Terminal 2 - set_fingers_position
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'bash', '-c',
                 'ros2 run leap_hand_control set_fingers_position; exec bash'],
            shell=False
        ),

        # Terminal 3 - read_sensors
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'bash', '-c',
                 'ros2 run leap_hand_control read_sensors ; exec bash'],
            shell=False
        ),
    ])
