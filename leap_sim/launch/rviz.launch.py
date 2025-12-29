from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import SetEnvironmentVariable, TimerAction, ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import Command, TextSubstitution, EnvironmentVariable,LaunchConfiguration
import os
from moveit_configs_utils.launches import generate_demo_launch
from moveit_configs_utils import MoveItConfigsBuilder

from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():

    ld = LaunchDescription()

    # Package paths (strings, not substitutions)
    leap_sim_share = get_package_share_directory("leap_sim")
    leap_desc_share = get_package_share_directory("leap_description")
    leap_moveit_share = get_package_share_directory("leap_moveit")
    leap_desc_root = os.path.dirname(leap_desc_share)  # .../share

    # Where your custom gz plugins might live (workspace install)
    # leap_sim_share = .../install/leap_sim/share/leap_sim
    leap_sim_prefix = os.path.dirname(os.path.dirname(leap_sim_share))  # .../install/leap_sim
    leap_sim_lib = os.path.join(leap_sim_prefix, "lib")

    robot_xacro = os.path.join(leap_moveit_share, "config", "leap.urdf.xacro")
    # robot_xacro = os.path.join(leap_desc_share, "assets", "leap_hand", "robot_gz.urdf.xacro")

    controllers_yaml = os.path.join(leap_sim_share, "config", "leap_controller.yaml")

    # Expand xacro -> URDF string
    robot_description = Command([
        "xacro", " ", robot_xacro,
        " ", "controllers_yaml:=", controllers_yaml, " ", "use_gazebo:=", "true"
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
    ld.add_action(DeclareLaunchArgument("publish_frequency", default_value="15.0"))

    # Given the published joint states, publish tf for the robot links and the robot description
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        respawn=True,
        output="screen",
        parameters=[
            {
                "publish_frequency": LaunchConfiguration("publish_frequency"),
            },
            {"robot_description": robot_description},
            {"use_sim_time": True}
        ],
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

    gz_driver = Node(
        package="leap_sim",
        executable="leap_gz_driver",
        output="screen",
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=[
            "/world/leap_world/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "--ros-args",
            "-r",
            "/world/leap_world/clock:=/clock",
        ],
    )

    ld.add_action(SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", gz_resource_path))
    ld.add_action(SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", gz_system_plugin_path))
    ld.add_action(SetEnvironmentVariable("GZ_SIM_PLUGIN_PATH", gz_plugin_path))

    robot_ld = LaunchDescription([
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", gz_resource_path),
        SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", gz_system_plugin_path),
        SetEnvironmentVariable("GZ_SIM_PLUGIN_PATH", gz_plugin_path),
        gz_sim,
        TimerAction(period=2.0, actions=[spawn_entity]),
        TimerAction(period=4.0, actions=[spawn_pos])
    ])


    ld.add_action(rsp_node)
    ld.add_action(clock_bridge)
    ld.add_action(SetParameter(name="use_sim_time", value=True))
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f"{leap_moveit_share}/launch/move_group.launch.py"
            ),
        )
    )
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f"{leap_moveit_share}/launch/moveit_rviz.launch.py"
            ),
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f"{leap_moveit_share}/launch/spawn_controllers.launch.py"
            ),
        )
    )
    ld.add_action(robot_ld)
    ld.add_action(TimerAction(period=10.0, actions=[gz_driver]))
    return ld