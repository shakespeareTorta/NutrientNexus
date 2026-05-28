import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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


def generate_launch_description():
    package_share = _package_share_or_source_dir()
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = PathJoinSubstitution([package_share, 'config', 'nav2_params.yaml'])

    turtlebot3_share = get_package_share_directory('turtlebot3_bringup')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    return LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation clock if true',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([turtlebot3_share, 'launch', 'robot.launch.py'])
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([slam_toolbox_share, 'launch', 'online_async_launch.py'])
            ),
            launch_arguments={'use_sim_time': 'False'}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([nav2_bringup_share, 'launch', 'navigation_launch.py'])
            ),
            launch_arguments={
                'use_sim_time': 'False',
                'params_file': params_file,
            }.items(),
        ),
        _zone_detector_action(use_sim_time),
    ])
