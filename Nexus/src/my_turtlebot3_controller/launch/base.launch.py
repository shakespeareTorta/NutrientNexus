#!/usr/bin/env python3
import os
import sys

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import AppendEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    # Get paths to needed packages
    launch_file_dir = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"), "launch"
    )
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    # ------------------------------------------------------------------
    # Locate nav2_simulation_params.yaml robustly across workspaces
    # ------------------------------------------------------------------
    nav2_params_file = None

    # Method 1: Sourced package share directory
    try:
        my_controller_pkg_share = get_package_share_directory(
            "my_turtlebot3_controller")
        candidate = os.path.join(
            my_controller_pkg_share, "config", "nav2_simulation_params.yaml")
        if os.path.exists(candidate):
            nav2_params_file = candidate
    except PackageNotFoundError:
        pass

    # Method 2: Relative to current launch file location
    if not nav2_params_file:
        launch_dir = os.path.dirname(os.path.realpath(__file__))
        candidate = os.path.normpath(
            os.path.join(launch_dir, "..", "config",
                         "nav2_simulation_params.yaml"))
        if os.path.exists(candidate):
            nav2_params_file = candidate

    # Method 3: System default fallback from nav2_bringup
    if not nav2_params_file:
        try:
            nav2_params_file = os.path.join(
                get_package_share_directory("nav2_bringup"),
                "params", "nav2_params.yaml")
        except PackageNotFoundError:
            pass
        print("[WARNING] Could not find 'nav2_simulation_params.yaml'. "
              "Using default nav2_bringup params.",
              file=sys.stderr)

    # Set launch arguments. Set initial position of robot
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    x_pose = LaunchConfiguration("x_pose", default="0.0")
    y_pose = LaunchConfiguration("y_pose", default="0.0")

    # ------------------------------------------------------------------
    # World file (graceful fallback if my_tb3_world is not built/sourced)
    # ------------------------------------------------------------------
    try:
        my_pkg_share = get_package_share_directory("my_tb3_world")
        world = os.path.join(my_pkg_share, "worlds", "new_world.world")
    except PackageNotFoundError:
        world = "empty.sdf"
        print("\n" + "=" * 72 + "\n"
              "[WARNING] package 'my_tb3_world' not found/sourced!\n"
              "Falling back to built-in 'empty.sdf' world.\n"
              "Build your 'my_tb3_world' package and source the workspace "
              "to use the custom agricultural field.\n" +
              "=" * 72 + "\n",
              file=sys.stderr)

    # Add TurtleBot3 models to Gazebo search path
    set_env_vars_resources = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.path.join(
            get_package_share_directory("turtlebot3_gazebo"), "models"),
    )

    # Launch Gazebo server with custom world
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": f"-r -s -v2 {world}",
            "on_exit_shutdown": "true",
        }.items(),
    )

    # Launch Gazebo client
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": "-g -v2",
            "on_exit_shutdown": "true",
        }.items(),
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "robot_state_publisher.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "spawn_turtlebot3.launch.py")
        ),
        launch_arguments={
            "x_pose": x_pose,
            "y_pose": y_pose,
        }.items(),
    )

    # Launch Nav2
    nav2_launch_args = {
        "use_sim_time": "True",
        "slam": "True",
        "cmd_vel_topic": "/cmd_vel_nav",
        "autostart": "True",
    }
    if nav2_params_file:
        nav2_launch_args["params_file"] = nav2_params_file

    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch",
                "navigation_launch.py",
            )
        ),
        launch_arguments=nav2_launch_args.items(),
    )

    # Launch SLAM
    slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("slam_toolbox"),
                "launch",
                "online_async_launch.py",
            )
        ),
        launch_arguments={"use_sim_time": "True"}.items(),
    )

    # ros_gz_bridge setup — bridges Gazebo Sim topics to ROS 2
    bridge_cmd = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(set_env_vars_resources)
    ld.add_action(bridge_cmd)
    ld.add_action(slam_cmd)
    ld.add_action(nav2_cmd)

    return ld
