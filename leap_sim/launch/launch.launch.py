from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Locate packages
    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim')
    leap_sim_pkg = FindPackageShare('leap_sim')            # your simulation package
    leap_description_pkg = FindPackageShare('leap_description')  # your robot models
    gz_launch_path = PathJoinSubstitution([ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py'])

    return LaunchDescription([
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
    ])