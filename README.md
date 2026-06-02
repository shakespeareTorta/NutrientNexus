# Building
To build, cd into the project root and issue:
```sh
. scripts/setup.sh
```

# Running
Make sure you have done steps outlined in [Building](#building) *in the current
shell*. This is required since the script sources necessary files for running
the project.

To start NutrientNexus, make sure the current directory is the project root and
issue (adjust to your shell):
```sh
ros2 launch my_turtlebot3_controller nexus.launch.py
```

If you get errors regarding not found packages, make sure that
`scripts/setup.sh` has been run.

# Technical details

This launch the launch file `nexus.launch.py`, located in
`src/my_turtlebot3_controller/launch/`.

It, in order, runs another launch file -- `base.launch.py` -- and also a number
of nodes:
- `safety_stop_node`
- `field_sensor_mock_node`
- `zone_detector_node`
- `robot_resource_node`
- `crop_decision_node`
- `dashboard_node`
- `sustainability_audit_node`
- `twin_supervisor_node`
- `rviz2`
- "rosbag2 auto-recording"

`base.launch.py` does the following:
- Looks for nav2_simulation_params.yml
- Starts gazebo using the world file
- Starts turtlebot3 in the simulation
- Starts and configures ROS2 <-> Gazebo bridge
- Brings up nav2
- Brings up SLAM
- Brings up robot_state_publisher from turtlebot3 ROS2 files
