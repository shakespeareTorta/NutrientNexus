#!/usr/bin/env python3
"""
Nutrient Nexus Dual-Robot Launch File (Scenario B: Cooperative Zone Division).

Spawns TWO TurtleBot3 robots in the same Gazebo world:
  Robot A (default namespace)  — patrols Zone 0 & 1  → future physical twin
  Robot B (/tb2 namespace)     — patrols Zone 2 & 3  → sim-only

Robot A uses the default namespace so its topics (/cmd_vel, /scan, /odom) are
identical to the physical TurtleBot3's. Switching from sim-only to physical
operation requires ZERO code changes — just stop Gazebo's Robot A and connect
the real robot.

Usage:
  ros2 launch my_turtlebot3_controller nexus_dual.launch.py
  ros2 launch my_turtlebot3_controller nexus_dual.launch.py gui:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap


def generate_launch_description():
    my_controller_share = get_package_share_directory("my_turtlebot3_controller")
    nexus_params_file = os.path.join(my_controller_share, 'config', 'nexus_params.yaml')

    # ── Launch Arguments ──────────────────────────────────────────────
    declare_gui_cmd = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Set to "false" to run headless (no Gazebo GUI).')
    gui = LaunchConfiguration("gui")

    # ── 1. Base infrastructure (Gazebo + Robot A + SLAM + Nav2) ────────
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_controller_share, "launch", "base.launch.py")),
        launch_arguments={
            'gui': gui,
            'x_pose': '0.6',
            'y_pose': '-1.6',
            'yaw': '-1.5708',   # Face outwards to the field
        }.items(),
    )

    # ── 2. Spawn Robot B ──────────────────────────────────────────────
    turtlebot3_model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    model_folder = 'turtlebot3_' + turtlebot3_model
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models', model_folder, 'model.sdf')

    spawn_tb2 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'tb2',
            '-file', urdf_path,
            '-x', '-0.7',       # Start in open space West
            '-y', '-0.5',
            '-z', '0.01',
            '-Y', '1.5708',     # Face North
            '-allow_renaming',  # Safety: rename if name conflict
        ],
        output='screen',
    )

    # ── 3. Bridge Robot B's Gazebo topics to ROS 2 under /tb2 ─────────
    #    Gazebo Harmonic prefixes plugin topics as /model/<name>/...
    tb2_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/tb2/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/tb2/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/tb2/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/model/tb2/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            ('/model/tb2/cmd_vel', '/tb2/cmd_vel'),
            ('/model/tb2/odometry', '/tb2/odom'),
            ('/model/tb2/scan', '/tb2/scan'),
            ('/model/tb2/joint_states', '/tb2/joint_states'),
        ],
        output='screen',
    )

    # ── 4. Robot B's robot_state_publisher (TF for tb2) ───────────────
    #    Isolate Robot B's whole TF onto /tb2/tf so its frame names
    #    (base_footprint, base_link, wheels, …) never collide with Robot A's
    #    on the global /tf. SetRemap redirects the RSP broadcaster's /tf and
    #    /tf_static; PushRosNamespace makes it read /tb2/joint_states.
    tb2_rsp = GroupAction([
        PushRosNamespace('tb2'),
        SetRemap('/tf', 'tf'),
        SetRemap('/tf_static', 'tf_static'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('turtlebot3_gazebo'),
                    'launch', 'robot_state_publisher.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),
    ])

    safety_stop_a = Node(
        package="my_turtlebot3_controller",
        executable="safety_stop_node",
        name="safety_stop_node",
        output="screen", emulate_tty=True,
        parameters=[
            nexus_params_file,
            {'use_sim_time': True},
            {'scan_topic': '/scan'},
            {'input_cmd_topic': '/cmd_vel_nav'},
            {'output_cmd_topic': '/cmd_vel'},
        ],
    )

    nav_exec_a = Node(
        package="my_turtlebot3_controller",
        executable="navigation_executor_node",
        name="navigation_executor_node",
        output="screen", emulate_tty=True,
        parameters=[{'use_sim_time': True}],
    )

    zone_detector_a = Node(
        package="my_turtlebot3_controller",
        executable="zone_detector_node",
        name="zone_detector_node",
        output="screen", emulate_tty=True,
        parameters=[{'use_sim_time': True}],
    )

    resource_a = Node(
        package="my_turtlebot3_controller",
        executable="robot_resource_node",
        name="robot_resource_node",
        output="screen", emulate_tty=True,
        parameters=[nexus_params_file, {'use_sim_time': True}, {'robot_id': 'A'}],
    )

    crop_decision_a = Node(
        package="my_turtlebot3_controller",
        executable="crop_decision_node",
        name="crop_decision_node_a",
        output="screen", emulate_tty=True,
        parameters=[nexus_params_file, {'use_sim_time': True}, {'robot_id': 'A'}, {'zone_assignment': [0, 1]}],
    )

    system_monitor_a = Node(
        package="my_turtlebot3_controller",
        executable="system_monitor_node",
        name="system_monitor_node_a",
        output="screen", emulate_tty=True,
        parameters=[nexus_params_file, {'use_sim_time': True}],
    )

    # ── 6. Robot B nodes (Zone 2 & 3) ─────────────────────────────────
    #    These use /tb2 topic remappings for navigation and sensing
    safety_stop_b = Node(
        package="my_turtlebot3_controller",
        executable="safety_stop_node",
        name="safety_stop_node_b",
        namespace="tb2",
        output="screen", emulate_tty=True,
        parameters=[
            nexus_params_file,
            {'use_sim_time': True},
            {'scan_topic': '/tb2/scan'},
            {'input_cmd_topic': '/tb2/cmd_vel_nav'},
            {'output_cmd_topic': '/tb2/cmd_vel'},
        ],
    )

    nav_exec_b = Node(
        package="my_turtlebot3_controller",
        executable="navigation_executor_node",
        name="navigation_executor_node_b",
        namespace="tb2",
        output="screen", emulate_tty=True,
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/dispatch_nav_goal', '/tb2/dispatch_nav_goal'),
            ('/navigation_executor_status', '/tb2/navigation_executor_status'),
            ('/navigate_to_pose', '/tb2/navigate_to_pose'),
        ],
    )

    zone_detector_b = Node(
        package="my_turtlebot3_controller",
        executable="zone_detector_node",
        name="zone_detector_node_b",
        namespace="tb2",
        output="screen", emulate_tty=True,
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/current_zone', '/tb2/current_zone'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
    )

    resource_b = Node(
        package="my_turtlebot3_controller",
        executable="robot_resource_node",
        name="robot_resource_node_b",
        namespace="tb2",
        output="screen", emulate_tty=True,
        parameters=[nexus_params_file, {'use_sim_time': True}, {'robot_id': 'B'}],
        remappings=[
            ('/odom', '/tb2/odom'),
            ('/robot_resources', '/tb2/robot_resources'),
            ('/refill_resources', '/tb2/refill_resources'),
        ],
    )

    crop_decision_b = Node(
        package="my_turtlebot3_controller",
        executable="crop_decision_node",
        name="crop_decision_node_b",
        namespace="tb2",
        output="screen", emulate_tty=True,
        parameters=[nexus_params_file, {'use_sim_time': True}, {'robot_id': 'B'}, {'zone_assignment': [2, 3]}],
        remappings=[
            ('/dispatch_nav_goal', '/tb2/dispatch_nav_goal'),
            ('/navigation_executor_status', '/tb2/navigation_executor_status'),
            ('/current_zone', '/tb2/current_zone'),
            ('/cmd_vel', '/tb2/cmd_vel'),
            ('/robot_resources', '/tb2/robot_resources'),
            ('/refill_resources', '/tb2/refill_resources'),
        ],
    )

    odom_to_tf_b = Node(
        package="my_turtlebot3_controller",
        executable="odom_to_tf_node",
        name="odom_to_tf_node_b",
        namespace="tb2",
        output="screen", emulate_tty=True,
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
    )

    system_monitor_b = Node(
        package="my_turtlebot3_controller",
        executable="system_monitor_node",
        name="system_monitor_node_b",
        namespace="tb2",
        output="screen", emulate_tty=True,
        parameters=[nexus_params_file, {'use_sim_time': True}],
        remappings=[
            ('/battery_state', '/tb2/battery_state'),
            ('/scan', '/tb2/scan'),
            ('/imu', '/tb2/imu'),
            ('/system_health', '/tb2/system_health'),
        ]
    )

    # ── 7. Robot B Nav2 ───────────────────────────────────────────────
    nav2_tb2_cmd = GroupAction([
        PushRosNamespace('tb2'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("nav2_bringup"), "launch", "navigation_launch.py")
            ),
            launch_arguments={
                "use_sim_time": "True",
                "use_namespace": "True",
                "namespace": "tb2",
                "cmd_vel_topic": "cmd_vel_nav",
                "autostart": "True",
            }.items(),
        )
    ])

    # Static TF map -> odom for Robot B, published on /tb2/tf_static so it
    # lives in B's isolated TF tree. Frame names match Nav2's defaults
    # (map / odom / base_link), so no per-robot Nav2 frame config is needed.
    tb2_tf_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tb2_static_tf',
        namespace='tb2',
        arguments=['-0.7', '-0.5', '0.01', '1.5708', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
    )

    # ── 8. Shared nodes (single instance for both robots) ─────────────
    field_sensor_mock = Node(
        package="my_turtlebot3_controller",
        executable="field_sensor_mock_node",
        name="field_sensor_mock_node",
        output="screen", emulate_tty=True,
        parameters=[nexus_params_file, {'use_sim_time': True}],
    )

    dashboard = Node(
        package="my_turtlebot3_controller",
        executable="dashboard_node",
        name="dashboard_node",
        output="screen", emulate_tty=True,
        parameters=[{'use_sim_time': True}],
    )

    sustainability_audit = Node(
        package="my_turtlebot3_controller",
        executable="sustainability_audit_node",
        name="sustainability_audit_node",
        output="screen", emulate_tty=True,
        parameters=[{'use_sim_time': True}],
    )

    # ── 8. Twin Supervisor (Lecture Week 6) ────────────────────────────
    twin_supervisor = Node(
        package="my_turtlebot3_controller",
        executable="twin_supervisor_node",
        name="twin_supervisor_node",
        output="screen", emulate_tty=True,
        parameters=[
            {'use_sim_time': True},
            {'robot_a_odom_topic': '/odom'},
            {'robot_b_odom_topic': '/tb2/odom'},
            {'system_mode': 'SIM_ONLY'},
        ],
    )

    # ── 9. rosbag2 auto-recording (Lecture Week 6: evidence collection) ─
    rosbag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '--output', '/tmp/nexus_recording',
            '--max-bag-duration', '300',  # 5-minute segments
            '/scan', '/odom', '/tf', '/cmd_vel',
            '/tb2/scan', '/tb2/odom', '/tb2/cmd_vel',
            '/field_moisture', '/field_nutrients', '/field_growth',
            '/robot_resources', '/tb2/robot_resources',
            '/sdg14_intervention', '/weather_forecast',
            '/sync_status', '/system_alerts', '/system_mode',
        ],
        output='screen',
    )

    # ── 10. RViz2 ─────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('nav2_bringup'),
            'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ── Assemble ──────────────────────────────────────────────────────
    return LaunchDescription([
        declare_gui_cmd,
        # Infrastructure
        base_launch,
        spawn_tb2,
        tb2_bridge,
        tb2_rsp,
        # Robot A nodes
        safety_stop_a,
        nav_exec_a,
        zone_detector_a,
        resource_a,
        crop_decision_a,
        system_monitor_a,
        # Robot B nodes
        safety_stop_b,
        nav_exec_b,
        zone_detector_b,
        resource_b,
        crop_decision_b,
        system_monitor_b,
        odom_to_tf_b,
        nav2_tb2_cmd,
        tb2_tf_cmd,
        # Shared nodes
        field_sensor_mock,
        dashboard,
        sustainability_audit,
        twin_supervisor,
        rosbag_record,
        rviz_node,
    ])
