from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command, TextSubstitution, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
import os



def generate_launch_description():
    # Locate packages
    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim')
    leap_sim_pkg = FindPackageShare('leap_sim')            # your simulation package
    leap_description_pkg = FindPackageShare('leap_description')  # your robot models
    gz_launch_path = PathJoinSubstitution([ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py'])

    leap_sim_share = get_package_share_directory("leap_sim")

    robot_xacro = PathJoinSubstitution([FindPackageShare("leap_description"), "assets/leap_hand", "robot.urdf.xacro"])

    controllers_yaml = os.path.join(
        get_package_share_directory("leap_sim"),
        "config",
        "leap_controller.yaml",
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "use_sim_time": True,
            "robot_description": Command([
                "xacro", " ", robot_xacro,
                " ", "controllers_yaml:=", controllers_yaml,
            ])
        }],
        output="screen",
    )

    # Spawn robot into Gazebo from /robot_descriptio

    # Load+activate controllers
    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )
    spawn_pos = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "-c", "/controller_manager"],
        output="screen",
    )
    
    spawn_gzd =  Node(
        package="leap_sim",
        executable="leap_gz_driver",
        output="screen",
    )

    return LaunchDescription([
        rsp,
        # Tell Gazebo where to find models and plugins
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            PathJoinSubstitution([
                leap_sim_pkg, 'models', ':', leap_description_pkg, 'assets/leap_hand'
            ])
        ),
        SetEnvironmentVariable(
            'GZ_SIM_PLUGIN_PATH',
            PathJoinSubstitution([leap_sim_pkg, 'plugins'])
        ),

        # Launch Gazebo with your world file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': PathJoinSubstitution([
                    leap_sim_pkg, 'worlds/world.sdf'
                ]),
                'on_exit_shutdown': 'True'
            }.items(),
        ),
        TimerAction(period=2.0, actions=[spawn_jsb, spawn_pos, spawn_gzd]),
    ])