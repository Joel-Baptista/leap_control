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

    ld = LaunchDescription()

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f"{get_package_share_directory("leap_moveit")}/launch/hand_motion_planning.launch.py"
            ),
        )
    )

    ld.add_action(
        TimerAction(period=5.0, 
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    f"{get_package_share_directory("leap_control")}/launch/launch.launch.py"
                ),
            )]
        )
    )


    

    return ld