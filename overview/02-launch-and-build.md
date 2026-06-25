# Launch & Build

The system is launched with a single command, but behind it sit two layered
launch files plus a handful of supporting scripts. This document maps the
boot sequence top-down and explains every option.

---

## TL;DR

```sh
# from repo root
. scripts/setup.sh                              # source ROS + colcon build
ros2 launch my_turtlebot3_controller nexus.launch.py
```

To run headless (no Gazebo GUI):

```sh
ros2 launch my_turtlebot3_controller nexus.launch.py gui:=false
```

---

## 1. `scripts/setup.sh`

```sh
export TURTLEBOT3_MODEL=burger
. /opt/ros/jazzy/setup.sh
[ "$1" != "--no-build" ] && colcon build --symlink-install
[ -e install/setup.sh ] && . install/setup.sh
```

- Sourcing `/opt/ros/jazzy/setup.sh` makes `ros2` etc. available.
- `colcon build --symlink-install` rebuilds only changed Python sources; no
  re-install needed for edits during development.
- The script is **sourced** (`. scripts/setup.sh`), not executed — that is what
  makes the env vars stick in the current shell.

---

## 2. `nexus.launch.py` (master bringup)

File: `src/my_turtlebot3_controller/launch/nexus.launch.py`

Argument:

| Name | Default | Meaning |
|---|---|---|
| `gui` | `true` | If `false`, Gazebo client is skipped (server + bridge still run) |

Boots in this order (see `LaunchDescription` in source):

| # | Action | What it brings up |
|---|---|---|
| 1 | `base.launch.py` (included) | Gazebo server + client, TB3 spawn, ROS↔Gazebo bridge, robot_state_publisher, ground-truth localization, Nav2 + SLAM |
| 2 | `safety_stop_node` | LiDAR collision guard (gates `/cmd_vel_raw` → `/cmd_vel`) |
| 3 | `field_sensor_mock_node` | Soil telemetry simulator |
| 4 | `weather_adapter_node` | Open-Meteo weather fetcher (optional, default lat/lon = 52,5) |
| 5 | `navigation_executor_node` | Nav2 action-client relay |
| 6 | `zone_detector_node` | TF-based physical zone verification |
| 7 | `robot_resource_node` | Battery + fertilizer tank |
| 8 | `crop_decision_node` | FSM brain + SDG-14 controller |
| 9 | `dashboard_node` | Tkinter GUI |
| 10 | `sustainability_audit_node` | Independent ledger |
| 11 | `twin_supervisor_node` | Task dispatcher + storm abort |
| 12 | `system_monitor_node` | Hardware watchdog (single publisher of `/system_health`) |
| 13 | `zone_visualizer_node` | Gazebo coloured tiles |
| 14 | `rviz2` | Default Nav2 view |
| 15 | `ros2 bag record` | Auto-recording of 15 key topics, 5 min window |

Every node is configured with `use_sim_time: True` so the simulation clock
from Gazebo is used. Most also receive `robot_id: 'A'` so the topics stay
relative to a future namespace.

The `base.launch.py` invocation passes:
```
gui: <arg>
x_pose: 0.6, y_pose: -1.6, yaw: 1.5708
```
which is the robot's spawn pose inside the L-shaped room.

---

## 3. `base.launch.py` (Gazebo + Nav2)

File: `src/my_turtlebot3_controller/launch/base.launch.py`

What it does, in source order:

1. **Locate `nav2_simulation_params.yaml`** with three fallback strategies:
   1. Sourced package share directory.
   2. Relative to the launch file (source-tree).
   3. Default `nav2_bringup/params/nav2_params.yaml`.
2. **Resolve the world** to `my_tb3_world/worlds/new_world.world`. Falls back
   to `empty.sdf` with a loud warning if the package is not built/sourced.
3. **Append `GZ_SIM_RESOURCE_PATH`** so Gazebo finds the TurtleBot3 models.
4. **Launch Gazebo server** with `gz_args='-r -s -v2 <world>'`
   (`-r` autostart, `-s` server-only when `gui:=false`).
5. **Launch Gazebo client** conditionally on `gui`.
6. **Spawn the TurtleBot3** directly via `ros_gz_sim create` with the
   configured pose. (`turtlebot3_gazebo`'s bundled `spawn_turtlebot3.launch.py`
   doesn't accept a yaw, so we call `ros_gz_sim create` ourselves.)
7. **ROS↔Gazebo bridge** — manually defined because the TB3 bridge file maps
   `/cmd_vel` to `TwistStamped` which breaks Nav2/teleop. The manual bridge
   maps:
   - `/cmd_vel`        ← `Twist` ↔ `gz.msgs.Twist`
   - `/odom`           ← `Odometry` ↔ `gz.msgs.Odometry`
   - `/scan`           ← `LaserScan` ↔ `gz.msgs.LaserScan`
   - `/tf`             ← `TFMessage` ↔ `gz.msgs.Pose_V`
   - `/joint_states`   ← `JointState` ↔ `gz.msgs.Model`
   - `/imu`            ← `Imu` ↔ `gz.msgs.IMU`
   - `/clock`          ← `Clock` ↔ `gz.msgs.Clock`
8. **robot_state_publisher** (from `turtlebot3_gazebo`'s launch).
9. **Nav2** (from `nav2_bringup/navigation_launch.py`):
   - `use_sim_time=True`, `slam=True`, `cmd_vel_topic=cmd_vel_raw`,
     `autostart=True`
   - `default_nav_to_pose_bt_xml` and `default_nav_through_poses_bt_xml` are
     pinned to the BT-navigator package's behaviour trees.
   - `params_file=nav2_simulation_params.yaml` (custom — see config).
10. **Ground-truth localization** (custom node) — replaces SLAM by
    publishing `map→odom` from Gazebo's true pose, so the `map` frame coincides
    with the world frame. SLAM is still brought up by Nav2 for completeness but
    its output is unused.

---

## 4. Docker

```sh
docker build -t nutrient_nexus .
docker run -it --rm \
    --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    nutrient_nexus
```

What the `Dockerfile` does:

| Step | Detail |
|---|---|
| Base | `osrf/ros:jazzy-desktop` |
| Env | `DEBIAN_FRONTEND=noninteractive`, `ROS_DISTRO=jazzy` |
| apt | `python3-pip`, `python3-colcon-common-extensions`, `ros-jazzy-navigation2`, `ros-jazzy-nav2-bringup`, `ros-jazzy-slam-toolbox`, `ros-jazzy-ros-gz`, `ros-jazzy-turtlebot3*` |
| pip | `flask --break-system-packages` (legacy dashboard hook) |
| Workspace | `/ros2_ws` |
| Copy | `src/` → `/ros2_ws/src/` |
| Build | `colcon build --symlink-install` (sourcing `/opt/ros/jazzy/setup.bash`) |
| Bashrc | auto-sources `/opt/ros/jazzy/setup.bash` + `/ros2_ws/install/setup.bash`, exports `TURTLEBOT3_MODEL=burger` |
| Default cmd | `bash` (interactive) |

GUI requires X11 forwarding (the run command above works on Linux hosts; macOS
needs `socat` shims).

---

## 5. Nix flake (optional dev shell)

```nix
# flake.nix
nix-ros-overlay + nixpkgs → mkShell with:
  - pkgs.colcon
  - ros.buildEnv with: ros-core, ament-cmake-core, python-cmake-module,
    ros-gz, ros-gz-sim, gz-launch-vendor, turtlebot3-{msgs,description,
    simulations,gazebo,navigation2,teleop}
shellHook:
  ROS_DISTRO=jazzy
  TURTLEBOT3_MODEL=burger
  unset QT_QPA_PLATFORM     # Gazebo is broken on Wayland
```

Plus the ROS binary cache at `https://ros.cachix.org` for hermetic builds.

To use it:

```sh
nix develop
# in the resulting shell:
colcon build --symlink-install
. install/setup.sh
ros2 launch my_turtlebot3_controller nexus.launch.py
```

---

## 6. Build outputs

- `build/`, `install/`, `log/` are colcon's standard outputs. They are
  gitignored.
- `build/` contains a `.built_by` marker and a top-level `COLCON_IGNORE` so
  the workspace itself is not double-built.
- `install/` contains the per-package setups and the global `setup.bash` /
  `setup.zsh` / `setup.sh`.

---

## 7. Re-build / re-run cycle

```sh
# after editing Python source — symlink-install means no rebuild needed:
. install/setup.sh
ros2 launch my_turtlebot3_controller nexus.launch.py

# after editing setup.py / package.xml / launch files / configs:
. scripts/setup.sh
ros2 launch my_turtlebot3_controller nexus.launch.py

# after editing nav2 params:
. scripts/setup.sh
```

Stop with `Ctrl-C` in the launch terminal — `on_exit_shutdown: true` cascades
to Gazebo.