from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml
import copy

def generate_launch_description():

    pkg_share = get_package_share_directory('leap_control')
    dynamixel_yaml_file = os.path.join(pkg_share, 'config', 'dynamixel.yaml')
        
    with open(dynamixel_yaml_file, 'r') as f:
        dynamixel_config = yaml.safe_load(f)

    nodes = []

    nodes.append(
        Node(
            package='leap_control',
            executable='leap_control_driver',
            name='leap_control_driver_node',
            parameters=[dynamixel_config],
            output='screen'
        )
    )

    return LaunchDescription(nodes)
