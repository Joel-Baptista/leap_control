from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml
import copy

def generate_launch_description():

    # Load YAML
    pkg_share = get_package_share_directory('leap_control')
    yaml_file = os.path.join(pkg_share, 'config', 'leap.yaml')
    dynamixel_yaml_file = os.path.join(pkg_share, 'confing', 'dynamixel.yaml')

    with open(yaml_file, 'r') as f:
        config = yaml.safe_load(f)
        
    with open(dynamixel_yaml_file, 'r') as f:
        dynamixel_config = yaml.safe_load(f)

    nodes = []

    # Create one node per name
    for finger in config['finger_nodes']:

        cfg = copy.deepcopy(config["finger_params"])
        cfg["finger_name"] = finger

        print(cfg)

        nodes.append(
            Node(
                package='leap_control',
                executable='finger_manager',  # SAME executable
                name=f"{finger}_node",
                parameters=[cfg],
                output='screen'
            )
        )
    # Append the main manager node
    nodes.append(
        Node(
            package='leap_control',
            executable='hand_manager',
            name='manager_node',
            parameters=[dynamixel_config],
            output='screen'
        )
    )    
    
    # nodes.append(
    #     Node(
    #         package='leap_control',
    #         executable='leap_control_driver',
    #         name='leap_control_driver_node',
    #         parameters=[dynamixel_config],
    #         output='screen'
    #     )
    # )

    return LaunchDescription(nodes)
