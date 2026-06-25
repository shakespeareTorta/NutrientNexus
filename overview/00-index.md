# NutrientNexus — System Overview

**NutrientNexus** is an autonomous, sustainable precision-agriculture digital
twin built on ROS 2 (Jazzy) and Gazebo Sim (Harmonic). A simulated TurtleBot3
patrols a custom L-shaped agricultural arena containing four crop zones and a
base station, monitors soil telemetry, and applies targeted irrigation and
fertilisation **only when safe** — the system is *sustainability-first* and
explicitly models the risk of nutrient runoff into waterways (SDG-14: Life Below
Water).

The project is also a teaching/assessment artefact for a "Digital Twin"
assignment (Option B: bidirectional pub/sub + state synchronisation + environment
interaction across the twin). It ships a Tkinter dashboard that simultaneously
**mirrors** the physical robot state and **injects** overrides (weather, faults)
that the robot reacts to.

---

## How to read this document

| File | What it covers |
|---|---|
| `00-index.md` *(this file)* | High-level summary, repo layout, how to run |
| `01-architecture.md` | System-level architecture, the digital-twin entities, runtime topology |
| `02-launch-and-build.md` | Launch files, the `nexus.launch.py` boot order, Dockerfile, Nix dev-shell |
| `03-nodes-reference.md` | Per-node reference (purpose, I/O, parameters, key logic) |
| `04-topic-contract.md` | Every topic in the system: type, direction, JSON schema |
| `05-arena-and-zones.md` | Gazebo world, zone bounding boxes, base station geometry |
| `06-decision-and-policy.md` | Crop decision FSM, sustainability rules, vulnerability model |
| `07-safety-and-faults.md` | SafetyStop, fault injection, weather scaling, scan staleness |
| `08-data-flows.md` | Walk-through of the three Option-B demo scripts |
| `09-configuration.md` | All YAML parameters, thresholds, defaults |

---

## What the system actually does

1. A TurtleBot3 Burger is spawned inside an L-shaped Gazebo room
   (`src/my_tb3_world/worlds/new_world.world`) with **four field zones at the
   four corners** and a **base station in the centre**.
2. The robot starts at the base station, asks the `TwinSupervisorNode` for its
   next assignment, and drives there using Nav2 (`NavigateToPose` action).
3. Once it arrives, `ZoneDetectorNode` confirms the robot's physical position
   (TF2 `map→base_footprint`) is inside the expected bounding box.
4. `CropDecisionNode` runs a finite-state machine: SCAN → DECIDE → ACTUATE.
   - It reads moisture / nutrients / vulnerability from `FieldSensorMockNode`.
   - **High runoff vulnerability ⇒ HALT fertilisation** (SDG-14 rule) and the
     refusal is published on `/sdg14_intervention` for the audit ledger.
   - Otherwise it irrigates / fertilises as needed (dynamically spinning in
     place to mime-spray, depleting fertilizer tank in `RobotResourceNode`).
5. After a short cooldown, the robot asks the supervisor for the next zone.
   When battery < 15 % or fertilizer < 10 %, it returns to base and the
   `RobotResourceNode` refills both.
6. **Bidirectional digital twin:** the `DashboardNode` (Tkinter) subscribes to
   robot telemetry (`/robot_resources`, `/system_health`, `/obstacle_status`,
   `/current_zone`, `/navigation_executor_status`) and **publishes** overrides
   back to the robot (`/weather_forecast`, `/twin_fault_state`). The robot's
   `SafetyStopNode` and `RobotResourceNode` react to those overrides.
7. `SustainabilityAuditNode` accumulates every fertilisation, irrigation and
   refused-intervention event in memory and, on demand, writes a Markdown
   report (`nexus_farm_report.md`).

---

## Repository layout

```
NutrientNexus/
├── README.md                         # Top-level build/run (very short)
├── Dockerfile                        # osrf/ros:jazzy-desktop + colcon build
├── flake.nix                         # Nix dev-shell for ROS 2 Jazzy
├── documentation.txt / main.tex      # LaTeX writeup of the project
├── options.txt / frames_*.pdf/.gv    # Slide frames for the demo
├── docs/README.md                    # Original project description
│
├── scripts/setup.sh                  # Source ROS + colcon build
│
├── src/
│   ├── my_tb3_world/                 # ament_cmake: ships the Gazebo world
│   │   └── worlds/new_world.world
│   │
│   └── my_turtlebot3_controller/     # ament_python: all custom nodes
│       ├── package.xml
│       ├── setup.py                  # 13 console_scripts entry points
│       ├── setup.cfg
│       ├── launch/
│       │   ├── nexus.launch.py       # Master bringup (everything)
│       │   └── base.launch.py        # Gazebo + Nav2 + ground-truth TF
│       ├── config/
│       │   ├── nexus_params.yaml     # Central tunable parameters
│       │   ├── zones.yaml            # Zone geometry & baselines
│       │   ├── nav2_simulation_params.yaml
│       │   └── slam_params.yaml
│       └── my_turtlebot3_controller/
│           ├── __init__.py
│           ├── qos.py                # Shared STATE_QOS profile
│           ├── lidar_utils.py        # Sector min / narrow-object helpers
│           ├── RobotResourceNode.py
│           ├── SystemMonitorNode.py
│           ├── algorithm/CropDecisionNode.py
│           ├── audit/SustainabilityAuditNode.py
│           ├── dashboard/DashboardNode.py
│           ├── localization/GroundTruthLocalizationNode.py
│           ├── navigation/
│           │   ├── NavigationExecutorNode.py
│           │   ├── SafetyStopNode.py
│           │   └── ZoneDetectorNode.py
│           ├── sensor/
│           │   ├── FieldSensorMockNode.py
│           │   └── WeatherAdapterNode.py
│           ├── twin/TwinSupervisorNode.py
│           └── visualization/ZoneVisualizerNode.py
│
├── build/  install/  log/           # colcon outputs (gitignored)
│
└── overview/                         # ← this document set
```

Two ROS 2 packages, one world package and one controller package with 13
custom nodes organised by concern (algorithm / audit / dashboard /
localization / navigation / sensor / twin / visualization).

---

## Runtime stack (one command)

```sh
. scripts/setup.sh                          # sources ROS + colcon build
ros2 launch my_turtlebot3_controller nexus.launch.py
```

`nexus.launch.py` boots, in order:
1. `base.launch.py` — Gazebo server + client, TurtleBot3 spawn, ROS↔Gazebo
   bridge, robot_state_publisher, ground-truth localization, Nav2 (with SLAM).
2. `safety_stop_node`         — LiDAR collision guard (defence-in-depth).
3. `field_sensor_mock_node`   — Soil telemetry simulation.
4. `weather_adapter_node`     — (optional) pulls Open-Meteo for live weather.
5. `navigation_executor_node` — Nav2 action-client relay.
6. `zone_detector_node`       — TF-based physical zone verification.
7. `robot_resource_node`      — Battery + fertilizer tank.
8. `crop_decision_node`       — FSM brain + SDG-14 controller.
9. `dashboard_node`           — Tkinter GUI (digital entity).
10. `sustainability_audit_node` — Independent ledger.
11. `twin_supervisor_node`    — Central task dispatcher / fault monitor.
12. `system_monitor_node`     — Hardware watchdog.
13. `zone_visualizer_node`    — Gazebo tiles that recolour with field state.
14. `rviz2`                   — Default Nav2 view.
15. `ros2 bag record`          — Auto-recording of 15 key topics (5 min window).

---

## Tech-stack at a glance

| Layer | Choice |
|---|---|
| OS / distro | Ubuntu 24.04, ROS 2 **Jazzy** |
| Simulator | Gazebo Sim **Harmonic** (`ros_gz`) |
| Robot | TurtleBot3 Burger (`turtlebot3_gazebo`, `turtlebot3_description`) |
| Navigation | Nav2 (`nav2_bringup`, DWB local planner, BT navigator) |
| Mapping | `slam_toolbox` (booted; localization overridden by ground-truth TF) |
| Bridge | `ros_gz_bridge` / `parameter_bridge` (manual, see `base.launch.py`) |
| Build | colcon (Python ament) |
| Container | `osrf/ros:jazzy-desktop` + Flask (pip, for legacy dashboard hook) |
| Reproducible env | Nix flake pinning ROS Jazzy + TB3 packages |
| Dashboard | Tkinter + ttk (no third-party viz) |
| External weather | Open-Meteo REST API (optional) |

---

## Where to look next

- **Want the big picture?** → `01-architecture.md`.
- **Want to run it?** → `02-launch-and-build.md`.
- **Want to read code?** → `03-nodes-reference.md`.
- **Want to know the wire format?** → `04-topic-contract.md`.
- **Want to tweak the rules?** → `06-decision-and-policy.md` + `09-configuration.md`.