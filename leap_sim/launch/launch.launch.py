from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, TimerAction, ExecuteProcess
from launch.substitutions import Command, TextSubstitution, EnvironmentVariable
import os


def generate_launch_description():
    # Package paths (strings, not substitutions)
    leap_sim_share = get_package_share_directory("leap_sim")
    leap_desc_share = get_package_share_directory("leap_description")
    leap_desc_root = os.path.dirname(leap_desc_share)  # .../share

    # Where your custom gz plugins might live (workspace install)
    # leap_sim_share = .../install/leap_sim/share/leap_sim
    leap_sim_prefix = os.path.dirname(os.path.dirname(leap_sim_share))  # .../install/leap_sim
    leap_sim_lib = os.path.join(leap_sim_prefix, "lib")

    robot_xacro = os.path.join(leap_desc_share, "assets", "leap_hand", "robot_gz.urdf.xacro")

    controllers_yaml = os.path.join(leap_sim_share, "config", "leap_controller.yaml")

    # Expand xacro -> URDF string
    robot_description = Command([
        "xacro", " ", robot_xacro,
        " ", "controllers_yaml:=", controllers_yaml,
    ])

    # Keep RSP alive regardless of Gazebo spawn attempts
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"use_sim_time": True},
            {"robot_description": robot_description},
        ],
        output="screen",
    )

    gz_resource_path = [
    TextSubstitution(text=os.path.join(leap_sim_share, "models")),
    TextSubstitution(text=":"),
    TextSubstitution(text=leap_desc_root),
    TextSubstitution(text=":"),
    EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
]

    gz_system_plugin_path = [
        TextSubstitution(text=leap_sim_lib),
        TextSubstitution(text=":"),
        TextSubstitution(text="/opt/ros/kilted/lib"),
        TextSubstitution(text=":"),
        EnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", default_value=""),
    ]

    gz_plugin_path = [
        TextSubstitution(text=os.path.join(leap_sim_share, "plugins")),
        TextSubstitution(text=":"),
        EnvironmentVariable("GZ_SIM_PLUGIN_PATH", default_value=""),
    ]

    gz_sim = ExecuteProcess(
        cmd=[
            "gz", "sim", "-v4",
            os.path.join(leap_sim_share, "worlds", "world.sdf"),
        ],
        output="screen",
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "leap_hand",
            "-topic", "robot_description",
            "-x", "0.0", "-y", "0.0", "-z", "0.1",
            # roll/pitch/yaw flags are -R -P -Y; you want yaw:
            "-R", "3.14159",
        ],
        output="screen",
    )

    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c", "/controller_manager",
            "--controller-manager-timeout", "120",
        ],
        output="screen",
    )

    spawn_pos = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "forward_position_controller",
            "-c", "/controller_manager",
            "--controller-manager-timeout", "120",
        ],
        output="screen",
    )

    return LaunchDescription([
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", gz_resource_path),
        SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", gz_system_plugin_path),
        SetEnvironmentVariable("GZ_SIM_PLUGIN_PATH", gz_plugin_path),
        rsp,
        gz_sim,
        TimerAction(period=6.0, actions=[spawn_entity]),
        TimerAction(period=14.0, actions=[spawn_jsb, spawn_pos]),
    ])
