#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Locate package share directory
    my_controller_share = get_package_share_directory("my_turtlebot3_controller")

    # 1. Include base.launch.py (starts Gazebo, SLAM, and Nav2)
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_controller_share, "launch", "base.launch.py")
        )
    )

    # 2. Field Sensor Mock Node
    field_sensor_mock = Node(
        package="my_turtlebot3_controller",
        executable="field_sensor_mock_node",
        name="field_sensor_mock_node",
        output="screen",
        emulate_tty=True,
    )

    # 3. Navigation Executor Node (Nav2 Action Client Relay)
    navigation_executor = Node(
        package="my_turtlebot3_controller",
        executable="navigation_executor_node",
        name="navigation_executor_node",
        output="screen",
        emulate_tty=True,
    )

    # 4. Crop Decision Node (Agricultural State Machine & SDG-14 controller)
    crop_decision = Node(
        package="my_turtlebot3_controller",
        executable="crop_decision_node",
        name="crop_decision_node",
        output="screen",
        emulate_tty=True,
    )

    # Return Launch Description containing all elements
    return LaunchDescription(
        [
            base_launch,
            field_sensor_mock,
            navigation_executor,
            crop_decision,
        ]
    )
