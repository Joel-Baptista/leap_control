from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import TimerAction
from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    leap_moveit_share = get_package_share_directory("leap_moveit")

    ld = LaunchDescription()

    ld.add_action(
        DeclareBooleanLaunchArg(
            "use_sim",
            default_value=False,
            description="By default, we launch the real robot drivers. Otherwise, launch Gazebo simulation",
        )
    )
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f"{leap_moveit_share}/launch/motion_planning.launch.py"
            ),
        )
    )

    ld.add_action(
        TimerAction(period=2.0, 
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    f"{get_package_share_directory("leap_sim")}/launch/launch.launch.py"
                ),
                condition=IfCondition(LaunchConfiguration("use_sim")),
            )]
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f"{get_package_share_directory("leap_control")}/launch/launch.launch.py"
            ),
            condition=UnlessCondition(LaunchConfiguration("use_sim")),
        )
    )    

    

    return ld