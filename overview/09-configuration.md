# Configuration

All tunable parameters live in YAML files under
`src/my_turtlebot3_controller/config/`. The launch file passes
`nexus_params.yaml` to every node that needs it; nodes ignore values
they don't declare.

---

## 1. `nexus_params.yaml` — central application parameters

This file is loaded by `nexus.launch.py` and passed to the application
nodes (safety_stop, system_monitor, crop_decision, robot_resource,
field_sensor_mock). The `/**:` ROS 2 YAML convention applies overrides
to every node in the same file.

### `safety_stop_node`
| Parameter | Default | Purpose |
|---|---|---|
| `stop_distance` | 0.22 m | Front hard-stop threshold |
| `narrow_obj_dist` | 0.18 m | Per-ray thin-object threshold |
| `front_angle_deg` | 15.0 | FRONT sector half-width |
| `side_angle_deg` | 70.0 | FRONT_LEFT/RIGHT/LEFT/RIGHT boundary |
| `rear_angle_deg` | 130.0 | SIDE/REAR boundary |
| `nudge_factor` | 0.4 | Multiplier on `nudge_turn_speed` |
| `nudge_turn_speed` | 0.45 rad/s | Pre-steering angular velocity |
| `scan_stale_sec` | 1.0 s | Block forward if scan older than this |

### `system_monitor_node`
| Parameter | Default | Purpose |
|---|---|---|
| `check_hz` | 2.0 | Health publish rate |
| `warn_voltage` | 11.5 V | Battery warning threshold |
| `crit_voltage` | 10.8 V | Battery critical threshold |
| `warn_percent` | 35.0 | Battery warning % |
| `crit_percent` | 20.0 | Battery critical % |
| `scan_stale_sec` | 2.0 s | LiDAR stale threshold (separate from SafetyStop) |
| `dropout_pct` | 0.40 | LiDAR high-dropout threshold |
| `imu_stale_sec` | 2.0 s | IMU stale threshold |
| `accel_warn_g` | 2.5 g | IMU accel alarm |
| `gyro_warn_rps` | 5.0 rad/s | IMU gyro alarm |

### `crop_decision_node`
| Parameter | Default | Purpose |
|---|---|---|
| `moisture_threshold` | 40.0 % | Irrigate below |
| `nutrient_threshold` | 50.0 % | Fertilise below |
| `vulnerability_halt_threshold` | 70.0 % | SDG-14 halt (high runoff) |
| `actuation_duration_sec` | 2.0 | Spin-in-place duration |
| `scan_duration_sec` | 2.0 | Time spent "sampling" before decide |
| `cooldown_duration_sec` | 1.0 | Pause between zones / sub-targets |
| `low_battery_threshold` | 15.0 % | Return-to-base threshold |
| `low_fertilizer_threshold` | 10.0 % | Return-to-base threshold |
| `battery_drain_per_meter` | 2.0 | Must match `robot_resource_node` |
| `battery_safety_margin` | 1.5 | Multiplier on round-trip estimate |

### `robot_resource_node`
| Parameter | Default | Purpose |
|---|---|---|
| `battery_drain_per_meter` | 2.0 %/m | Must match `crop_decision_node` |
| `fertilizer_drain_per_spray` | 15.0 % | Tank drain per fertilisation event |

### `field_sensor_mock_node`
| Parameter | Default | Purpose |
|---|---|---|
| `sim_tick_interval` | 2.0 s | Telemetry tick rate |
| `irrigate_replenish_pct` | 95.0 | Moisture after irrigation |
| `fertilise_replenish_pct` | 90.0 | Nutrients after fertilisation |

---

## 2. `zones.yaml` — zone geometry & baselines

```yaml
base_station:
  target_x: -0.1, target_y: 0.0, target_theta: 0.0
  min_x: -0.40, max_x: 0.20
  min_y: -0.30, max_y: 0.30
  baseline_moisture: 50.0, baseline_nutrients: 50.0
  runoff_risk: "Low"

zone_0: { target_x: -0.50, target_y: 0.55, target_theta: 0.0,
          min_x: -0.85, max_x: -0.15, min_y: 0.20, max_y: 0.90,
          baseline_moisture: 50.0, baseline_nutrients: 20.0,
          runoff_risk: "Low" }
zone_1: { target_x: 2.05, target_y: 0.55, target_theta: 180.0,
          min_x: 1.70, max_x: 2.40, min_y: 0.20, max_y: 0.90,
          baseline_moisture: 95.0, baseline_nutrients: 30.0,
          runoff_risk: "High" }
zone_2: { target_x: 2.05, target_y: -3.10, target_theta: 180.0,
          min_x: 1.70, max_x: 2.40, min_y: -3.45, max_y: -2.75,
          baseline_moisture: 20.0, baseline_nutrients: 80.0,
          runoff_risk: "Medium" }
zone_3: { target_x: -0.50, target_y: -3.10, target_theta: 0.0,
          min_x: -0.85, max_x: -0.15, min_y: -3.45, max_y: -2.75,
          baseline_moisture: 50.0, baseline_nutrients: 45.0,
          runoff_risk: "High" }
```

The `target_*` is where the robot drives; the `min_*` / `max_*` are the
bounding box that `ZoneDetectorNode` tests against. Tile / RViz markers
are anchored at `target_*` (zone centre) and sized from the bounding box.

---

## 3. `nav2_simulation_params.yaml` — Nav2 overrides

This file is passed to `nav2_bringup`'s `navigation_launch.py`. Key
overrides vs. the upstream default:

| Where | Change | Why |
|---|---|---|
| `bt_navigator.ros__parameters.transform_tolerance` | 0.5 → 1.0 s | "absorbs SLAM map→odom lag/stalls under WSL" |
| `controller_server.FollowPath.transform_tolerance` | 0.2 → 1.0 s | The original "Unable to transform robot pose into global plan's frame" error was caused by SLAM's 0.4 s transform going stale during 1.6 s map rebuilds |
| `controller_server.goal_checker.xy_goal_tolerance` | 0.25 (kept) | Allows the goal checker to accept close-enough arrivals |
| `controller_server.FollowPath.critics` | `["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]` | Standard DWB local planner set |

Everything else is the Jazzy default. See the file for the full list.

---

## 4. `slam_params.yaml` — SLAM toolbox overrides

`mode: mapping` (mapping, not pure localization). Notable settings:

| Parameter | Value | Note |
|---|---|---|
| `minimum_time_interval` | 0.5 s | SLAM processes a scan this often |
| `map_update_interval` | 5.0 s | Map rebuild cadence |
| `transform_timeout` | 1.0 s | Was 0.2 s — same rationale as Nav2's `transform_tolerance` |
| `resolution` | 0.05 m | 5 cm grid |
| `do_loop_closing` | true | |
| `transform_publish_period` | 0.02 s | |

SLAM output is *unused* in simulation — `GroundTruthLocalizationNode`
overrides `map→odom` with the gz ground truth. SLAM is still booted
because Nav2's `slam:=True` flag wants it; the cost of running it
during the demo is negligible.

---

## 5. Launch arguments

| Argument | Default | Where |
|---|---|---|
| `gui` | `true` | `nexus.launch.py` and `base.launch.py` (forwarded). `false` = no Gazebo client |

`base.launch.py` is invoked from `nexus.launch.py` with:
```
x_pose: 0.6
y_pose: -1.6
yaw:    1.5708   # 90°, facing the room interior
```

---

## 6. Environment variables

| Variable | Value | Set by | Used by |
|---|---|---|---|
| `TURTLEBOT3_MODEL` | `burger` | `scripts/setup.sh`, `flake.nix`, `Dockerfile` | `base.launch.py` chooses the model SDF |
| `ROS_DISTRO` | `jazzy` | `flake.nix`, `Dockerfile`, `scripts/setup.sh` | ROS 2 install path |
| `GZ_SIM_RESOURCE_PATH` | appended with TB3 models dir | `base.launch.py` | Gazebo model lookup |
| `QT_QPA_PLATFORM` | unset | `flake.nix` | "Gazebo is currently broken on Wayland" |

---

## 7. Where to change what (cheat sheet)

| You want to… | Edit |
|---|---|
| Make the robot drive further before returning to base | `crop_decision_node.low_battery_threshold` |
| Stop the SDG-14 halt firing so often | `crop_decision_node.vulnerability_halt_threshold` (raise) **and** `field_sensor_mock_node` weather factors in `FieldSensorMockNode.deplete_and_grow_tick` |
| Speed up / slow down the FSM | `crop_decision_node.scan_duration_sec`, `actuation_duration_sec`, `cooldown_duration_sec` |
| Change stop distance or pre-steer aggressiveness | `safety_stop_node.stop_distance`, `nudge_factor`, `nudge_turn_speed` |
| Add a new zone | `config/zones.yaml` + (optionally) tweak `FieldSensorMockNode` seeding |
| Move the base station | `zones.yaml` `base_station.target_x/y` + `nexus.launch.py` spawn pose + `base_station.min_x/max_x/min_y/max_y` |
| Use real weather | leave `weather_adapter_node` enabled, set its `latitude` / `longitude` (currently unconfigured in the launch file) |
| Make Nav2 more tolerant of slow TF | `nav2_simulation_params.yaml` `transform_tolerance` |
| Make the demo more aggressive on a slow machine | lower the `slam_toolbox` resolution, raise `transform_timeout` further |