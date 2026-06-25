# Nodes Reference

Every custom node, in alphabetical order. Each entry covers:
* purpose,
* subscribes / publishes,
* key parameters (with default and source of truth),
* the most important algorithmic detail.

> File path convention: `src/my_turtlebot3_controller/my_turtlebot3_controller/<subpkg>/<Node>.py`

---

## 1. `crop_decision_node`

**File:** `algorithm/CropDecisionNode.py`
**Class:** `CropDecisionNode`  **Executable:** `crop_decision_node`
**Backed by:** the `algorithm` subpackage — this is the brain.

### Purpose
The robot's decision brain. Runs a finite-state machine that asks the
supervisor for a zone, dispatches Nav2, verifies arrival via TF, scans,
decides, actuates (irrigate/fertilise) and cools down before the next zone.

### Subscribes
| Topic | Type | Notes |
|---|---|---|
| `navigation_executor_status` | `String` | From `NavigationExecutor` |
| `current_zone` | `String` | From `ZoneDetector` |
| `/field_moisture` | `Float32MultiArray` | Global, indexed by `ordered_zones` |
| `/field_nutrients` | `Float32MultiArray` | Global |
| `/field_vulnerability` | `Float32MultiArray` | Global, drives the SDG-14 halt |
| `/supervisor/zone_assignment` | `String` (JSON, STATE_QOS) | Only honoured in `WAITING_FOR_ASSIGNMENT` |
| `/system_alerts` | `String` (JSON, STATE_QOS) | `{"action":"ABORT", …}` → return to base |
| `robot_resources` | `String` (JSON, STATE_QOS) | `{battery, fertilizer}` |
| `odom` | `Odometry` | For predictive battery check |

### Publishes
| Topic | Type | Notes |
|---|---|---|
| `dispatch_nav_goal` | `PoseStamped` | Map frame goal for Nav2 |
| `irrigate_zone` | `String` (STATE_QOS) | JSON `{"robot":…, "zone":…}` |
| `fertilise_zone` | `String` (STATE_QOS) | JSON `{"robot":…, "zone":…}` |
| `cmd_vel_nav` | `Twist` | Used for spin-in-place actuation (treatment mimer) |
| `refill_resources` | `String` (STATE_QOS) | `"refill"` on arrival at base |
| `sdg14_intervention` | `String` (STATE_QOS) | `{"robot":…, "zone":…, "action":"HALT_FERTILIZER", "reason":…}` |
| `supervisor/zone_request` | `String` | JSON `{"robot":…}` |

### Parameters
All declared via `declare_parameter` and overridable through `nexus_params.yaml`
under the `crop_decision_node` namespace.

| Name | Default | Purpose |
|---|---|---|
| `robot_id` | `A` | For multi-robot routing |
| `moisture_threshold` | 40.0 | Below this → irrigate |
| `nutrient_threshold` | 50.0 | Below this → fertilise |
| `vulnerability_halt_threshold` | 70.0 | Above this → **HALT fertilisation** (SDG-14) |
| `actuation_duration_sec` | 2.0 | How long the spin-in-place "spray" lasts |
| `scan_duration_sec` | 2.0 | Time spent "sampling" before decide |
| `cooldown_duration_sec` | 1.0 | Pause between zones / sub-targets |
| `low_battery_threshold` | 15.0 | Return-to-base threshold |
| `low_fertilizer_threshold` | 10.0 | Return-to-base threshold |
| `battery_drain_per_meter` | 2.0 | Must match `robot_resource_node` |
| `battery_safety_margin` | 1.5 | Multiplier on round-trip estimate (Nav2 path overhead) |

### Key logic

**FSM phases (string in `self.current_phase`):**

```
IDLE → WAITING_FOR_ASSIGNMENT → NAVIGATING → VERIFYING_ZONE
   → SCANNING → DECIDING → ACTUATING → COOLDOWN → (loop or IDLE)
   ↘ RETURNING_TO_BASE ↗  (on low battery / low fertilizer / ABORT)
```

Invariants:
- At most one of `_scan_timer`, `_actuation_timer`, `_cooldown_timer` is
  active at any time. They are cancelled via `_cancel_and_destroy(...)`
  before the next phase's timer is created.
- `nav2_ready` is latched `True` on the first `IDLE` status from
  `NavigationExecutor` — before that, the FSM is parked.

**Predictive battery check** (`_can_afford_trip`):
Euclidean distance current→zone→base × `battery_drain_per_meter` ×
`battery_safety_margin` must be < current battery level. A failed check
diverts the robot home.

**Decide** (`_on_scan_complete`):
1. If zone is `base_station` → cooldown (no actuation).
2. If `vulnerability > vulnerability_halt_threshold` → publish
   `sdg14_intervention = HALT_FERTILIZER`, do **not** fertilise.
3. If `nutrients < nutrient_threshold` → publish `fertilise_zone`.
4. If `moisture < moisture_threshold` → publish `irrigate_zone`.
5. If at least one action was taken → `ACTUATING` (spin in place for
   `actuation_duration_sec` publishing `Twist(angular.z=0.6)`).
6. Otherwise → `COOLDOWN`.

**Resource guard** (top of `state_machine_tick`): if `battery ≤ low_battery` or
`fertilizer ≤ low_fertilizer`, all timers are cancelled, twist zeroed, and the
robot is dispatched to base regardless of current phase.

**Base dispatch** (`_dispatch_to_base`): sends the `base_station` target
with a per-robot Y-offset (±0.4 m) so multi-robot parking doesn't collide.

---

## 2. `dashboard_node`

**File:** `dashboard/DashboardNode.py`
**Class:** `DashboardNode`  **Executable:** `dashboard_node`

### Purpose
The **digital entity** of the twin. A 980×880 Tkinter window that mirrors
every important piece of robot state and lets the operator inject weather
and fault events that the physical side reacts to.

### Subscribes (Physical → Digital)
| Topic | Type | Mirrored to UI |
|---|---|---|
| `/robot_resources` | `String` (JSON) | Battery + Fertilizer progress bars |
| `/current_zone` | `String` | "Zone: …" label |
| `/navigation_executor_status` | `String` | "Nav: …" label |
| `/system_health` | `String` (JSON) | Health panel + TWIN MODE banner |
| `/obstacle_status` | `String` (JSON) | Obstacle panel (turns red when blocked) |
| `/field_moisture` | `Float32MultiArray` | Live zone table |
| `/field_nutrients` | `Float32MultiArray` | Live zone table |
| `/field_vulnerability` | `Float32MultiArray` | Live zone table (AT RISK → red) |

### Publishes (Digital → Physical)
| Topic | Type | When |
|---|---|---|
| `/weather_forecast` | `String` (STATE_QOS) | When operator clicks Sunny/Rainy/Overcast/Storm |
| `/twin_fault_state` | `String` (JSON, STATE_QOS) | When operator toggles LiDAR / Motor / Battery |
| `/generate_report` | `String` | When operator clicks "Generate Sustainability Report" |

### Parameters
| Name | Default |
|---|---|
| `robot_id` | `A` |
| `use_sim_time` | `True` |

### GUI panels (top-to-bottom)
1. **TWIN MODE banner** — green/amber/red, mirrors `twin_mode` from
   `/system_health` (`NORMAL`/`DEGRADED`/`FAULTED`).
2. **Physical Robot Resources** — Battery + Fertilizer progress bars, turn
   red < 30 % / < 20 %.
3. **Telemetry** — current zone, current nav status.
4. **Field Zone Status** — one live row per zone, colour-coded by category
   (HEALTHY, NEEDS WATER, NEEDS FERTILISER, NEEDS BOTH, AT RISK).
5. **System Health** — battery / LiDAR / IMU status mirrors.
6. **Environment / Obstacle** — single line, red on `blocked=true`.
7. **Weather Injection** — four buttons (Sunny, Rainy, Overcast, Storm).
8. **Fault Injection** — LiDAR cycle button (OK → DEGRADED → FAILED),
   Motor toggle (OK ↔ STALLED), Clear All Faults.
9. **Sustainability Audit** — "Generate Sustainability Report" button.

### Threading
ROS spin runs on a **daemon thread**; Tk `mainloop` runs on the main thread.
Callbacks only mutate a guarded `state` dict (`threading.Lock`); the
`update_gui_loop` re-paints every 100 ms from snapshots under the lock.

### Colour thresholds (mirror of `CropDecisionNode`)
`moisture_threshold = 40`, `nutrient_threshold = 50`,
`vulnerability_halt = 70`.

---

## 3. `field_sensor_mock_node`

**File:** `sensor/FieldSensorMockNode.py`

### Purpose
Simulates the agricultural field. Holds per-zone state (moisture,
nutrients, growth, vulnerability), evolves it with the weather, and
replenishes it on irrigate/fertilise events. Publishes the four telemetry
arrays the rest of the twin consumes.

### Subscribes
| Topic | Type |
|---|---|
| `/irrigate_zone` | `String` (STATE_QOS) |
| `/fertilise_zone` | `String` (STATE_QOS) |
| `/weather_forecast` | `String` (STATE_QOS) |

### Publishes
| Topic | Type | Format |
|---|---|---|
| `/field_moisture` | `Float32MultiArray` | one float per zone, sorted by zone id |
| `/field_nutrients` | `Float32MultiArray` | same |
| `/field_growth` | `Float32MultiArray` | same |
| `/field_vulnerability` | `Float32MultiArray` | same |

### Parameters
| Name | Default |
|---|---|
| `sim_tick_interval` | 2.0 s |
| `irrigate_replenish_pct` | 95.0 (moisture after irrigate) |
| `fertilise_replenish_pct` | 90.0 (nutrients after fertilise) |
| `robot_id` | `A` |

### Per-tick dynamics (one bullet per weather)
| Weather | Moisture | Nutrients |
|---|---|---|
| `rainy` | +1..3 | −0.3..0.8 (leaching) |
| `storm` | +3..6 | −1..2.5 (severe leaching) |
| `sunny` | −0.5..1.5 | −0.1..0.4 |
| `overcast` | −0.1..0.4 | −0.1..0.4 |

Clamps: moisture ∈ [5, 99], nutrients ≥ 5. Growth increments by 0.05..0.2
per tick when both moisture and nutrients > 30 %.

### Vulnerability model
```
vulnerability = min(100, base_risk × weather_factor × moisture × 100)
```
where `base_risk ∈ {Low:0.2, Medium:0.5, High:0.8}` (from `zones.yaml`) and
`weather_factor ∈ {rainy:1.5, storm:2.0, overcast:0.8, sunny:0.3}`. So a
"High" zone under storm with wet soil saturates the 0..100 scale — the SDG-14
halt threshold (70) is reached predictably.

---

## 4. `ground_truth_localization`

**File:** `localization/GroundTruthLocalizationNode.py`

### Purpose
Replaces SLAM in simulation. Streams the robot's true Gazebo pose from the
gz topic `/world/<world>/dynamic_pose/info` and publishes a `map→odom` TF
so the `map` frame coincides with the world frame.

### Why
`base.launch.py` comment is explicit: "slam_toolbox drifts badly in this
small, symmetric room". The global costmap is built from `/scan` (no static
map layer), so dropping SLAM costs nothing for planning. The pose stream
parses `gz.msgs.Pose` frames and looks for the `name = "burger"` entity.

### Threading
The pose is read by a daemon thread that runs `gz topic -e` as a subprocess
and parses the stream line-by-line. The TF broadcast happens on a 50 Hz
timer (the main thread). Until the first true pose arrives, identity
`map→odom` is published (correct at spawn).

---

## 5. `navigation_executor_node`

**File:** `navigation/NavigationExecutorNode.py`

### Purpose
Thin wrapper around the Nav2 `navigate_to_pose` action. Turns goals arriving
on `dispatch_nav_goal` into action requests and republishes the outcome as a
simple text status on `navigation_executor_status`.

### Subscribes
| Topic | Type |
|---|---|
| `dispatch_nav_goal` | `PoseStamped` |

### Publishes
| Topic | Type | Notes |
|---|---|---|
| `navigation_executor_status` | `String` (STATE_QOS) | One of: `IDLE`, `NAVIGATING`, `SUCCEEDED_AT_POSE`, `FAILED_NAVIGATION`, `ABORTED_NAVIGATION`, `CANCELED_NAVIGATION`, `REJECTED`, `IDLE_SERVER_UNAVAILABLE` |

A 0.5 s re-publish timer keeps late subscribers and transient-local clients
in sync with the latest status.

### Status transitions
```
NEW GOAL → wait_for_action_server (2s timeout)
   ├─ server unavailable → IDLE_SERVER_UNAVAILABLE
   └─ goal sent → NAVIGATING
        ├─ accepted → get_result_async
        │     ├─ SUCCEEDED → SUCCEEDED_AT_POSE
        │     ├─ ABORTED  → ABORTED_NAVIGATION
        │     └─ CANCELED → CANCELED_NAVIGATION
        └─ rejected → REJECTED
(any outcome) → scheduled 0.2s → IDLE
```

The 0.2 s delay before `IDLE` is intentional: it lets the brain observe the
terminal status before seeing the parked state.

---

## 6. `robot_resource_node`

**File:** `RobotResourceNode.py` (top-level, sibling of subpackages)

### Purpose
Simulates the robot's battery and fertilizer tank. Tracks the robot's
distance driven via `/odom` and drains battery accordingly. On
`fertilise_zone` (JSON addressed to this robot) it drains fertilizer. On
`refill_resources` it tops both up to 100 %. Publishes state as JSON on
`robot_resources` at 1 Hz.

### Class invariant
```
0.0 ≤ self.battery ≤ 100.0
0.0 ≤ self.fertilizer ≤ 100.0
```
Every mutator preserves this — drains are `max(0.0, ...)` and refills set
exactly 100.0.

### Subscribes
| Topic | Type |
|---|---|
| `odom` | `Odometry` |
| `fertilise_zone` | `String` |
| `refill_resources` | `String` |
| `/twin_fault_state` | `String` (JSON, STATE_QOS) |

### Publishes
| Topic | Type | Format |
|---|---|---|
| `robot_resources` | `String` (STATE_QOS) | `{"battery": …, "fertilizer": …}` (1 dp) |

### Parameters
| Name | Default |
|---|---|
| `battery_drain_per_meter` | 2.0 |
| `fertilizer_drain_per_spray` | 15.0 |
| `robot_id` | `A` |

### Battery-fault override (digital-twin)
When the dashboard sets `battery=clamped`:
- normal drain is suspended,
- the battery is clamped to `battery_fault_clamp = 10.0`,
- refill will only restore fertilizer (battery stays clamped).

This cascades into `CropDecisionNode`'s resource guard and into
`SystemMonitorNode` setting `twin_mode=FAULTED`.

---

## 7. `safety_stop_node`

**File:** `navigation/SafetyStopNode.py`

### Purpose
Last-line safety filter between the command mux and the robot. Five-sector
LiDAR scan, with stale-scan detection, weather velocity scaling, injected
fault reaction, and pre-steering nudge.

### Sectors (angles in degrees, half-widths from forward axis)
| Sector | Range | Behaviour |
|---|---|---|
| FRONT | ±15° | hard-stop below `stop_distance` (0.22 m) |
| FRONT_LEFT | +15°..+70° | pre-steering nudge right when narrow |
| FRONT_RIGHT | −70°..−15° | pre-steering nudge left when narrow |
| LEFT | +70°..+130° | open-side assessment |
| RIGHT | −130°..−70° | open-side assessment |

### Subscribes
| Topic | Type | Notes |
|---|---|---|
| `/scan` | `LaserScan` (BEST_EFFORT) | |
| `/cmd_vel_raw` (default) | `Twist` | Nav2 + treatment mux upstream |
| `/weather_forecast` | `String` (STATE_QOS) | sunny / rainy / storm |
| `/twin_fault_state` | `String` (JSON, STATE_QOS) | injected lidar/motor faults |

### Publishes
| Topic | Type | Notes |
|---|---|---|
| `/cmd_vel` (default) | `Twist` | Gated command to the robot |
| `/obstacle_status` | `String` (STATE_QOS, JSON) | `{"blocked", "distance", "sector"}` |

### Parameters
| Name | Default |
|---|---|
| `scan_topic` | `/scan` |
| `input_cmd_topic` | `/cmd_vel_raw` |
| `output_cmd_topic` | `/cmd_vel` |
| `stop_distance` | 0.22 m |
| `narrow_obj_dist` | 0.18 m |
| `front_angle_deg` | 15.0 |
| `side_angle_deg` | 70.0 |
| `rear_angle_deg` | 130.0 |
| `nudge_factor` | 0.4 |
| `nudge_turn_speed` | 0.45 rad/s |
| `scan_stale_sec` | 1.0 |

### Behaviour, in priority order
1. **Motor fault = stalled** → publish zero twist, obstacle=MOTOR_FAULT.
2. **Weather scaling** on `linear.x` (`rainy` × 0.6, `storm` × 0.4).
3. **LiDAR fault = failed** → forward blocked (rotation allowed),
   obstacle=LIDAR_FAULT.
4. **LiDAR fault = degraded** → `linear.x *= 0.5`, stop distance × 1.5.
5. **Scan stale** (`age > scan_stale_sec`) or never seen → forward blocked,
   obstacle=SCAN_STALE.
6. **Front blocked** (`d_front < stop_distance`) and forward requested →
   zero forward, allow rotation, obstacle=FRONT.
7. **Pre-steering nudge** when a diagonal sector is closing in but FRONT is
   still clear (and the upstream command isn't already turning): add a small
   `±nudge_turn_speed × nudge_factor` to `angular.z` and tag the obstacle
   sector.
8. Otherwise → pass-through, obstacle=CLEAR.

`obstacle_status` is de-duplicated — identical payloads are not republished.

---

## 8. `sustainability_audit_node`

**File:** `audit/SustainabilityAuditNode.py`

### Purpose
Independent ledger. Logs every fertilise, irrigate and SDG-14 intervention
(plus the weather at the time) and, on request, writes a Markdown report
to `nexus_farm_report.md` in the cwd.

### Subscribes
| Topic | Type |
|---|---|
| `/weather_forecast` | `String` |
| `/fertilise_zone` | `String` |
| `/irrigate_zone` | `String` |
| `/sdg14_intervention` | `String` (JSON) |
| `/generate_report` | `String` |

### Output
`nexus_farm_report.md` with sections:
* **SDG-14 Environmental Impact** — total prevented runoff events, estimated
  nitrogen saved (1.5 kg per intervention), intervention ledger.
* **Agricultural Operations** — totals.
* **AI Farm Recommendations** — zones with ≥ 2 interventions get a
  "drainage / relocation" recommendation.

---

## 9. `system_monitor_node`

**File:** `SystemMonitorNode.py` (top-level)

### Purpose
Hardware watchdog. The **single authoritative publisher** of
`/system_health`. Fuses raw sensor telemetry (battery, LiDAR, IMU) with
dashboard-injected faults into one JSON view plus a derived `twin_mode`.

### Subscribes
| Topic | Type |
|---|---|
| `/battery_state` | `BatteryState` (real robot; in sim returns NO_DATA) |
| `/scan` | `LaserScan` (BEST_EFFORT) |
| `/imu` | `Imu` (real robot; in sim returns NO_DATA) |
| `/twin_fault_state` | `String` (JSON, STATE_QOS) |

### Publishes
| Topic | Type | Notes |
|---|---|---|
| `/system_health` | `String` (STATE_QOS) | Per-subsystem status + `faults` block + `twin_mode` |

### Parameters
| Name | Default |
|---|---|
| `check_hz` | 2.0 |
| `warn_voltage` / `crit_voltage` | 11.5 / 10.8 |
| `warn_percent` / `crit_percent` | 35 / 20 |
| `scan_stale_sec` | 2.0 |
| `dropout_pct` | 0.40 |
| `imu_stale_sec` | 2.0 |
| `accel_warn_g` | 2.5 |
| `gyro_warn_rps` | 5.0 |

### Derived `twin_mode`
* `FAULTED` if any of: motor=stalled, lidar=failed, battery=clamped,
  battery raw=CRITICAL.
* `DEGRADED` if any of: lidar=degraded, lidar raw in
  {STALE, HIGH_DROPOUT, ALL_INVALID}, battery raw=WARNING.
* `NORMAL` otherwise.

Injected faults override the raw sensor verdict (e.g. `lidar=failed` is
honoured even if the raw scan looks fine).

---

## 10. `twin_supervisor_node`

**File:** `twin/TwinSupervisorNode.py`

### Purpose
Central orchestrator. Owns the patrol queue, dispatches the next zone to
the robot on request, watches for storms and physical faults, and
broadcasts a global abort when needed.

### Patrol queue
`['zone_2', 'zone_1', 'zone_3', 'zone_0']` — popped from the front and
re-queued at the back for continuous patrol.

### Subscribes
| Topic | Type |
|---|---|
| `/weather_forecast` | `String` (STATE_QOS) |
| `/robot_resources` | `String` (STATE_QOS) |
| `/navigation_executor_status` | `String` (STATE_QOS) |
| `/supervisor/zone_request` | `String` |

### Publishes
| Topic | Type |
|---|---|
| `/supervisor/zone_assignment` | `String` (JSON, STATE_QOS) — `{"robot", "zone"}` |
| `/sync_status` | `String` (JSON, STATE_QOS) — 1 Hz world snapshot |
| `/system_alerts` | `String` (JSON, STATE_QOS) — `{"action":"ABORT", "reason":…}` |

### Parameters
| Name | Default |
|---|---|
| `system_mode` | `SIM_ONLY` (passed by launch; HYBRID is reserved) |
| `robot_id` | `A` |
| `nav_fault_threshold` | 3 (consecutive failures → declare physical fault) |

### Behaviour
* **Storm** → broadcast `STORM_EMERGENCY` abort, then in the next request
  dispatch `BASE`.
* **Battery < 20 %** → next request dispatches `BASE`.
* **Battery < 15 %** (as reported by the resource callback) → pause
  operations immediately (`_handle_robot_fault`).
* **Navigation failure streak** — `_consecutive_nav_failures` increments on
  any failure status, **resets to 0 on a single `SUCCEEDED_AT_POSE`** (so a
  transient abort cannot trigger an abort storm), and triggers
  `_handle_robot_fault` once it reaches the threshold. The streak check is
  edge-triggered (only counts state changes, not re-publications).
* **Faulted robot** — zone requests are ignored until `battery ≥ 20 %` AND
  nav status is `IDLE` (recover-and-resume).

---

## 11. `weather_adapter_node`

**File:** `sensor/WeatherAdapterNode.py`

### Purpose
Optional bridge to a real weather API. Periodically queries Open-Meteo for
the configured coordinates and republishes a categorised weather state
(`sunny` / `overcast` / `rainy` / `storm`) on `/weather_forecast`.

### Parameters
| Name | Default |
|---|---|
| `latitude` | 52.0 |
| `longitude` | 5.0 |
| `update_interval_sec` | 300.0 |

### WMO-code → category
| WMO | Category |
|---|---|
| ≥ 95 | storm |
| 51..82 | rainy |
| 3 | overcast |
| 1..2 | sunny |
| 45..48 (fog) | overcast |
| else | sunny |

Network and parse errors are caught (URLError, TimeoutError, JSONDecodeError,
KeyError, TypeError, ValueError) and only logged. This node is purely
advisory; the dashboard's manual buttons are the primary weather source
during a demo.

---

## 12. `zone_detector_node`

**File:** `navigation/ZoneDetectorNode.py`

### Purpose
Reports the robot's current zone and (optionally) draws RViz markers.
Looks up `map→base_footprint` via TF2, tests against the bounding boxes in
`config/zones.yaml`, publishes the result on `/current_zone`. Loads
`zones.yaml` from the installed share dir, falling back to the source
directory if the package is not built/sourced.

### Subscribes
(none — pulls TF and loads YAML at init)

### Publishes
| Topic | Type | Notes |
|---|---|---|
| `/current_zone` | `String` (STATE_QOS) | zone name or `no_zone` |
| `/zone_markers` | `MarkerArray` | one CUBE per zone (current zone highlighted) + one SPHERE per target |

### Timers
* 0.5 s — `update_and_publish_zone`.
* 1.0 s — `publish_markers`.

### Tie-break rule
Multiple zone boxes can overlap (e.g. base_station and zone_2). When that
happens, the **alphabetically smallest** zone name wins. The base station
is therefore deliberately the smallest in alphabetical order so it beats
overlapping field zones.

### Markers
Cubes are centred on the bounding-box centre, sized to the extents
(min 0.1 m), drawn at z = 0.025, with a 0.8-alpha highlight for the
current zone. Targets are 0.2 m white spheres at z = 0.1.

---

## 13. `zone_visualizer_node`

**File:** `visualization/ZoneVisualizerNode.py`

### Purpose
Paints each field zone as a semi-transparent coloured tile in Gazebo, with
the colour matching the live state (same thresholds as
`CropDecisionNode`). The tile is anchored at the zone's navigation target
— exactly where the robot drives to treat that zone.

### Subscribes
| Topic | Type |
|---|---|
| `/field_moisture` | `Float32MultiArray` (STATE_QOS) |
| `/field_nutrients` | `Float32MultiArray` (STATE_QOS) |
| `/field_vulnerability` | `Float32MultiArray` (STATE_QOS) |

### Publishes
(none — drives Gazebo via the gz service `/world/<world>/create` and
`/world/<world>/remove`.)

### Parameters
| Name | Default |
|---|---|
| `world_name` | `default` |
| `moisture_threshold` | 40.0 |
| `nutrient_threshold` | 50.0 |
| `vulnerability_halt_threshold` | 70.0 |
| `update_period_sec` | 1.0 |
| `tile_alpha` | 0.70 |
| `tile_height` | 0.02 m |

### Legend
| Colour | Category |
|---|---|
| Green | healthy |
| Blue | needs water |
| Yellow | needs fertiliser |
| Orange | needs both |
| Red | at risk (SDG-14 halt) |
| Grey | no telemetry yet |

A zone is only re-spawned when its colour category actually changes (no
churn). On a clean shutdown all tiles are removed.