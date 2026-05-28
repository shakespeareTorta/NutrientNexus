import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _package_share_or_source_dir() -> str:
    try:
        return get_package_share_directory('nutrient_nexus_navigation')
    except Exception:
        # Allow launch files to run directly from the workspace before install/setup is sourced.
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _zone_detector_action(use_sim_time):
    try:
        get_package_share_directory('nutrient_nexus_navigation')
        return Node(
            package='nutrient_nexus_navigation',
            executable='zone_detector',
            name='zone_detector',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        )
    except Exception:
        script_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'nutrient_nexus_navigation',
            'zone_detector.py',
        )
        return ExecuteProcess(
            cmd=[sys.executable, script_path, '--ros-args', '-p', ['use_sim_time:=', use_sim_time]],
            name='zone_detector',
            output='screen',
        )


def generate_launch_description() -> LaunchDescription:
    package_share = _package_share_or_source_dir()
    turtlebot3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    world_file = os.path.join(package_share, 'worlds', 'farm_box.world')
    nav2_params_file = os.path.join(package_share, 'config', 'nav2_params.yaml')

    return LaunchDescription(
        [
            SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='True',
                description='Use simulation clock for all nodes.',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(turtlebot3_gazebo_share, 'launch', 'turtlebot3_world.launch.py')
                ),
                launch_arguments={
                    'world': world_file,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(slam_toolbox_share, 'launch', 'online_async_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': nav2_params_file,
                }.items(),
            ),
            _zone_detector_action(use_sim_time),
        ]
    )
