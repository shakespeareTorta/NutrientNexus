import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    my_custom_turtlebot3_gazebo_launch_file = os.path.join(
        get_package_share_directory('my_custom_turtlebot3_gazebo'), 'launch', 'empty_world.launch.py'
    )

    slam_toolbox_launch_file = os.path.join(
        get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py'
    )
    
    nav2_bringup_launch_file = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py'
    )

    rviz_config_file = os.path.join(
        get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz'
    )

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(my_custom_turtlebot3_gazebo_launch_file),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox_launch_file),
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_bringup_launch_file),
        ),
    ])