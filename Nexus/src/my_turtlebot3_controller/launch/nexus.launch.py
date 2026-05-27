#!/usr/bin/env python3
"""
Unified Nutrient Nexus Launch File.

Starts the entire agricultural digital twin simulation pipeline with
a single command:
  1. base.launch.py  — Gazebo + SLAM + Nav2
  2. SafetyStopNode  — LiDAR collision guard (defence-in-depth)
  3. FieldSensorMock  — Soil telemetry simulation
  4. NavigationExecutor — Nav2 action client relay
  5. CropDecisionNode — Agricultural state machine & SDG-14 controller

Usage:
  ros2 launch my_turtlebot3_controller nexus.launch.py
  ros2 launch my_turtlebot3_controller nexus.launch.py gui:=false  # headless
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Locate package share directory
    my_controller_share = get_package_share_directory("my_turtlebot3_controller")

    # Declare gui argument (passed through to base.launch.py)
    declare_gui_cmd = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to "false" to run headless (no Gazebo GUI).'
    )
    gui = LaunchConfiguration("gui")

    # 1. Include base.launch.py (starts Gazebo, SLAM, and Nav2)
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_controller_share, "launch", "base.launch.py")
        ),
        launch_arguments={'gui': gui}.items(),
    )

    # 2. Safety Stop Node — LiDAR-based collision guard (defence-in-depth)
    #    Intercepts /cmd_vel_nav from Nav2 and publishes safe commands to /cmd_vel
    safety_stop = Node(
        package="my_turtlebot3_controller",
        executable="safety_stop_node",
        name="safety_stop_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {'scan_topic': '/scan'},
            {'input_cmd_topic': '/cmd_vel_nav'},
            {'output_cmd_topic': '/cmd_vel'},
            {'stop_distance': 0.30},
            {'front_angle_deg': 30.0},
        ],
    )

    # 3. Field Sensor Mock Node
    field_sensor_mock = Node(
        package="my_turtlebot3_controller",
        executable="field_sensor_mock_node",
        name="field_sensor_mock_node",
        output="screen",
        emulate_tty=True,
    )

    # 4. Navigation Executor Node (Nav2 Action Client Relay)
    navigation_executor = Node(
        package="my_turtlebot3_controller",
        executable="navigation_executor_node",
        name="navigation_executor_node",
        output="screen",
        emulate_tty=True,
    )

    # 5. Crop Decision Node (Agricultural State Machine & SDG-14 controller)
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
            declare_gui_cmd,
            base_launch,
            safety_stop,
            field_sensor_mock,
            navigation_executor,
            crop_decision,
        ]
    )
