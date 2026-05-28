# NutrientNexus Week 1 Navigation Foundation

## Project Overview

This repository currently contains the Week 1 navigation foundation for the `nutrient_nexus_navigation` ROS2 package. The implemented scope focuses on TurtleBot3 Burger navigation infrastructure for ROS2 Jazzy in Gazebo simulation and on a real robot bringup path.

What is in place today:

- A Python ROS2 package named `nutrient_nexus_navigation`
- Simulation and real-robot launch files
- A Nav2 parameter file tuned for an initial TurtleBot3 Burger setup
- A simple zone detector node that maps odometry positions to three configured crop zones
- An RViz configuration for navigation visualization
- A package world file at `nutrient_nexus_navigation/worlds/farm_box.world`

The current simulation world was adopted from `docs/SimFiles/new_world.world` and adapted for the package workflow. For the current package, Gazebo Sim plugin blocks were removed and obstacle models were made static so the world can be reused through the existing TurtleBot3 launch path.

This is a navigation foundation only. It does not yet implement nutrient decisions, custom message types, dashboards, or sensor-processing logic.

## Prerequisites

- Linux environment with `nix` available for the provided development shell
- ROS2 Jazzy environment
- TurtleBot3 Burger model
- Gazebo-based TurtleBot3 simulation packages used by this repository
- A colcon workspace rooted at this project

The current shell setup in `flake.nix` exports:

- `ROS_DISTRO=jazzy`
- `TURTLEBOT3_MODEL=burger`

The package depends on the ROS stack already referenced by the workspace, including:

- `nav2_bringup`
- `slam_toolbox`
- `turtlebot3_gazebo`
- `turtlebot3_bringup`
- `turtlebot3_description`

## Installation

1. Enter the project root:

```bash
cd <workspace-root>
```

2. Start the provided ROS2 Jazzy development shell:

```bash
nix develop
```

3. Build the navigation package:

```bash
colcon build --packages-select nutrient_nexus_navigation
```

4. Source the workspace:

```bash
source install/setup.bash
```

Expected result: the package builds without errors and generates `install/setup.bash`.

5. Confirm the package is visible:

```bash
ros2 pkg prefix nutrient_nexus_navigation
```

Expected result: prints the install prefix for `nutrient_nexus_navigation`.

## Configuration

Current package configuration lives under `nutrient_nexus_navigation/`:

- `config/nav2_params.yaml` - initial Nav2, AMCL, planner, controller, and costmap parameters
- `config/zones.yaml` - three axis-aligned crop zones: `zone_a`, `zone_b`, and `zone_c`
- `launch/sim_bringup.launch.py` - simulation bringup using Gazebo, SLAM Toolbox, Nav2, and the zone detector
- `launch/real_bringup.launch.py` - real robot bringup using TurtleBot3 bringup, SLAM Toolbox, Nav2, and the zone detector
- `rviz/nav.rviz` - RViz layout for map, scan, robot, costmaps, and paths
- `worlds/farm_box.world` - adapted world file used by simulation launch

Important current behavior:

- Simulation launch sets `use_sim_time=True`
- Real robot launch defaults to `use_sim_time=False`
- Both launch files hardcode `TURTLEBOT3_MODEL=burger`
- Zone detection subscribes to `/odom` and publishes `/current_zone` as `std_msgs/String`

## Usage - Simulation

After building and sourcing the workspace, start the simulation stack:

```bash
ros2 launch nutrient_nexus_navigation sim_bringup.launch.py
```

Expected result: Gazebo starts, the TurtleBot3 world launches, Nav2 and SLAM nodes appear, and `/current_zone` becomes available.

What this currently starts:

- TurtleBot3 Gazebo launch
- `slam_toolbox` online async launch
- Nav2 navigation launch with `config/nav2_params.yaml`
- `zone_detector` node

Useful follow-up commands:

```bash
ros2 node list
ros2 topic list
ros2 topic echo /current_zone
```

Expected result: node list includes `zone_detector`, topic list includes `/scan`, `/map`, `/cmd_vel`, and `/current_zone`, and the echo prints a zone label or `no_zone`.

The simulation launch uses `nutrient_nexus_navigation/worlds/farm_box.world`, which is the package-local world adapted from `docs/SimFiles/new_world.world`.

## Usage - Real Robot

For the real robot path, build and source the workspace on the robot or on the machine connected to it, then run:

```bash
ros2 launch nutrient_nexus_navigation real_bringup.launch.py
```

Expected result: bringup starts without launch-time errors and the robot-side stack exposes `/odom`, `/scan`, and `/current_zone`.

What this currently wires together:

- TurtleBot3 real robot bringup
- `slam_toolbox` online async launch
- Nav2 navigation launch with the package Nav2 parameters
- `zone_detector` node

Helpful checks:

```bash
ros2 node list
ros2 topic echo /current_zone
ros2 topic echo /odom
```

Expected result: node list includes `zone_detector`, `/odom` publishes pose updates, and `/current_zone` changes when the robot moves between configured areas.

This README only documents the existing bringup path. It does not claim validated autonomous field performance beyond the current launch integration.

## Testing

The repository currently includes an integration-style launch test at `nutrient_nexus_navigation/test/test_sim_navigation.py`.

Build, source, and run the package tests with:

```bash
colcon test --packages-select nutrient_nexus_navigation
colcon test-result --verbose
```

Additional lightweight verification commands:

```bash
python3 -m compileall nutrient_nexus_navigation
ros2 pkg prefix nutrient_nexus_navigation
```

The current test coverage described here is limited to what is present in the repository today.

## Topics

Current package-relevant ROS topics visible from the implemented files include:

| Topic | Direction | Purpose |
| --- | --- | --- |
| `/odom` | subscribe | Consumed by `zone_detector` for position-based zone lookup |
| `/current_zone` | publish | Current zone name or `no_zone` |
| `/scan` | upstream input | LIDAR input used by SLAM and Nav2 costmaps |
| `/map` | upstream output | SLAM-generated map consumed by Nav2 and RViz |
| `/cmd_vel` | upstream output | Velocity commands emitted by Nav2 / teleop toward the robot base |
| `/clock` | upstream output | Simulation clock when `use_sim_time=True` |

Additional Nav2 and SLAM topics are brought in by upstream launch files, but the table above lists the main topics expected during Week 1 bringup and verification.

## Troubleshooting

- If `ros2` commands cannot find the package, rerun `source install/setup.bash` after `colcon build`
- If the environment lacks ROS2 Jazzy tools, start from `nix develop` before building or launching
- If simulation fails to load the expected environment, verify that `nutrient_nexus_navigation/worlds/farm_box.world` exists and that `sim_bringup.launch.py` still points to it
- If `/current_zone` stays at `no_zone`, check that odometry is being published on `/odom` and that the robot is inside one of the configured bounds in `config/zones.yaml`
- If Gazebo display behavior is problematic on Wayland, note that `flake.nix` currently unsets `QT_QPA_PLATFORM`

## Architecture

Current Week 1 package layout:

```text
<workspace-root>
├── README.md
├── flake.nix
└── nutrient_nexus_navigation/
    ├── config/
    │   ├── nav2_params.yaml
    │   └── zones.yaml
    ├── launch/
    │   ├── real_bringup.launch.py
    │   └── sim_bringup.launch.py
    ├── nutrient_nexus_navigation/
    │   ├── __init__.py
    │   └── zone_detector.py
    ├── package.xml
    ├── rviz/
    │   └── nav.rviz
    ├── setup.py
    ├── test/
    │   ├── test_sim_navigation.py
    │   └── test_zone_detection.py
    └── worlds/
        └── farm_box.world
```

Runtime flow today:

1. A launch file starts simulation or real-robot bringup.
2. Upstream TurtleBot3, SLAM Toolbox, and Nav2 launch descriptions are included.
3. The package `zone_detector` node subscribes to `/odom`.
4. The node checks the current position against `config/zones.yaml` bounds.
5. The node publishes the matching zone name to `/current_zone`.

This architecture is intentionally small and direct for Week 1.
