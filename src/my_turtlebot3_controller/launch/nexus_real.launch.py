#!/usr/bin/env python3
"""
Nutrient Nexus Real-Robot Launch File.

Runs the full digital twin pipeline against a physical TurtleBot3 Burger.
Gazebo mirrors the robot's state for visualization; the real robot is the
source of truth for odometry, LiDAR, and TF.

Prerequisites (on the TurtleBot3 Raspberry Pi):
  export ROS_DOMAIN_ID=30
  export TURTLEBOT3_MODEL=burger
  ros2 launch turtlebot3_bringup robot.launch.py

Then on this workstation:
  export ROS_DOMAIN_ID=30
  source install/setup.bash
  ros2 launch my_turtlebot3_controller nexus_real.launch.py

To build a new map instead of using the saved one, pass slam:=true:
  ros2 launch my_turtlebot3_controller nexus_real.launch.py slam:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    my_pkg = get_package_share_directory('my_turtlebot3_controller')

    declare_slam = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Run SLAM to build a new map. Set false to localise with AMCL on the saved map.')

    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='True',
        description='Launch Gazebo GUI for digital twin visualisation.')

    slam = LaunchConfiguration('slam')
    gui = LaunchConfiguration('gui')

    nav2_params = os.path.join(my_pkg, 'config', 'nav2_real_params.yaml')
    map_file = os.path.join(my_pkg, 'maps', 'big_map.yaml')

    bt_navigator_share = get_package_share_directory('nav2_bt_navigator')
    bt_xml_nav_to_pose = os.path.join(
        bt_navigator_share, 'behavior_trees',
        'navigate_to_pose_w_replanning_and_recovery.xml')
    bt_xml_nav_through_poses = os.path.join(
        bt_navigator_share, 'behavior_trees',
        'navigate_through_poses_w_replanning_and_recovery.xml')

    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    my_tb3_world_share = get_package_share_directory('my_tb3_world')
    world = os.path.join(my_tb3_world_share, 'worlds', 'new_world.world')

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f'-r -s -v2 {world}',
            'on_exit_shutdown': 'false',
        }.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-g -v2'}.items(),
        condition=IfCondition(gui),
    )

    # Bridge: forward /cmd_vel to Gazebo so the twin mirrors real motion.
    # Real robot topics (/scan, /odom, /tf) come from the physical robot over
    # the network — do NOT bridge them back from Gazebo.
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
        output='screen',
    )

    nav2_with_map = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': 'False',
            'autostart': 'True',
            'map': map_file,
            'params_file': nav2_params,
            'cmd_vel_topic': '/cmd_vel_nav',
            'default_nav_to_pose_bt_xml': bt_xml_nav_to_pose,
            'default_nav_through_poses_bt_xml': bt_xml_nav_through_poses,
        }.items(),
        condition=UnlessCondition(slam),
    )

    nav2_with_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'False',
            'slam': 'True',
            'autostart': 'True',
            'params_file': nav2_params,
            'cmd_vel_topic': '/cmd_vel_nav',
            'default_nav_to_pose_bt_xml': bt_xml_nav_to_pose,
            'default_nav_through_poses_bt_xml': bt_xml_nav_through_poses,
        }.items(),
        condition=IfCondition(slam),
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch', 'online_async_launch.py')),
        launch_arguments={'use_sim_time': 'False'}.items(),
        condition=IfCondition(slam),
    )

    safety_stop = Node(
        package='my_turtlebot3_controller',
        executable='safety_stop_node',
        name='safety_stop_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'scan_topic': '/scan'},
            {'input_cmd_topic': '/cmd_vel_nav'},
            {'output_cmd_topic': '/cmd_vel'},
            {'stop_distance': 0.30},
            {'front_angle_deg': 30.0},
        ],
    )

    field_sensor_mock = Node(
        package='my_turtlebot3_controller',
        executable='field_sensor_mock_node',
        name='field_sensor_mock_node',
        output='screen',
        emulate_tty=True,
    )

    navigation_executor = Node(
        package='my_turtlebot3_controller',
        executable='navigation_executor_node',
        name='navigation_executor_node',
        output='screen',
        emulate_tty=True,
    )

    zone_detector = Node(
        package='my_turtlebot3_controller',
        executable='zone_detector_node',
        name='zone_detector_node',
        output='screen',
        emulate_tty=True,
    )

    robot_resource = Node(
        package='my_turtlebot3_controller',
        executable='robot_resource_node',
        name='robot_resource_node',
        output='screen',
        emulate_tty=True,
    )

    crop_decision = Node(
        package='my_turtlebot3_controller',
        executable='crop_decision_node',
        name='crop_decision_node',
        output='screen',
        emulate_tty=True,
    )

    dashboard = Node(
        package='my_turtlebot3_controller',
        executable='dashboard_node',
        name='dashboard_node',
        output='screen',
        emulate_tty=True,
    )

    sustainability_audit = Node(
        package='my_turtlebot3_controller',
        executable='sustainability_audit_node',
        name='sustainability_audit_node',
        output='screen',
        emulate_tty=True,
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('nav2_bringup'),
            'rviz', 'nav2_default_view.rviz')],
        output='screen',
    )

    return LaunchDescription([
        declare_slam,
        declare_gui,
        gzserver,
        gzclient,
        gz_bridge,
        nav2_with_map,
        nav2_with_slam,
        slam_toolbox,
        safety_stop,
        field_sensor_mock,
        navigation_executor,
        zone_detector,
        robot_resource,
        crop_decision,
        dashboard,
        sustainability_audit,
        rviz,
    ])
