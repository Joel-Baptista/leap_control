import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from launch.actions import RegisterEventHandler, TimerAction, LogInfo
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, TimerAction, ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import Command, TextSubstitution, EnvironmentVariable,LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_dir = get_package_share_directory("leap_sim")
    leap_sim_share = get_package_share_directory("leap_sim")
    world_description_path = os.path.join(package_dir, "worlds", "leap_world.urdf")
    leap_moveit_share = get_package_share_directory("leap_moveit")

    robot_xacro = os.path.join(leap_moveit_share, "config", "leap.urdf.xacro")
    # robot_xacro = os.path.join(leap_desc_share, "assets", "leap_hand", "robot_gz.urdf.xacro")

    controllers_yaml = os.path.join(leap_moveit_share, "config", "ros2_controllers.yaml")


    # Expand xacro -> URDF string
    robot_description = Command([
        "xacro", " ", robot_xacro,
        " ", "controllers_yaml:=", controllers_yaml, " ", "use_gazebo:=", "false"
    ])

    webots = WebotsLauncher(
        world=os.path.join(package_dir, "worlds", "world.wbt"),
    )

    my_robot_driver = WebotsController(
        robot_name="Leap",
        parameters=[
            {"robot_description": world_description_path},
        ],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True, "robot_description": robot_description}],
        arguments=[robot_description],
    )

    exit_event = RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        )
    )

    move_group = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f"{leap_moveit_share}/launch/move_group.launch.py"
            ),
        )
    
    rviz = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f"{leap_moveit_share}/launch/moveit_rviz.launch.py"
            ),
        )
    
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_yaml]
    )


    controllers = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f"{leap_moveit_share}/launch/spawn_controllers.launch.py"
            ),
        )
    

    return LaunchDescription(
        [
            webots,
            my_robot_driver,
            TimerAction(period=3.0, actions=[robot_state_publisher_node]),
            move_group,
            rviz,
            controller_manager,
            controllers,
            exit_event,
        ]
    )
