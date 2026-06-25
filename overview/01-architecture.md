# Architecture

This document is the system-level picture: entities, responsibilities,
boundaries, and the message flows that hold the system together.

---

## 1. The two-entity digital twin (Option B)

NutrientNexus is built around **two clearly separated entities** that stay
synchronised through ROS 2 topics. This is the core architectural decision and
it satisfies the three Option-B requirements explicitly.

| Entity | Substrate | Role | Source of truth for |
|---|---|---|---|
| **Physical entity** | TurtleBot3 Burger in Gazebo (Harmonic) | Drives, senses, actuates | World state, sensor telemetry, robot pose |
| **Digital entity** | Tkinter GUI inside the `dashboard_node` | Mirrors + injects overrides | Operator UI, fault/weather injection |

**Mental model:**

```
                ┌─────────────────────────────────────────────┐
                │            Gazebo world (truth)            │
                └──────────┬──────────────────────┬───────────┘
       sensor/state       │                      │      cmd_vel
       ▼                  ▼                      ▼        ▲
  /scan, /odom,    /robot_resources,      /system_health,  │
  /imu, /tf,       /obstacle_status,      /current_zone,   │
  /battery_state   /navigation_executor_status               │
       │                  │                      │           │
       │       ┌──────────┴──────────────────────┘           │
       │       │                                              │
       │       ▼                                              │
       │  Physical-side ROS nodes (the robot's "nervous system")
       │  (safety_stop, resources, monitor, decision, …)
       │       │
       │       │ /weather_forecast, /twin_fault_state,
       │       │ /system_alerts, /supervisor/zone_assignment
       │       ▼
       │  ┌─────────────────────────────────────────┐
       └─►│  DashboardNode (Tkinter) — digital side │
          │  + TwinSupervisorNode + SustainabilityAudit
          └─────────────────────────────────────────┘
```

### Option-B requirement → mechanism

1. **Bidirectional pub/sub.**
   P→D: `/robot_resources`, `/system_health`, `/obstacle_status`,
   `/navigation_executor_status`, `/current_zone`, `/field_*` arrays.
   D→P: `/weather_forecast`, `/twin_fault_state`, `/generate_report`.

2. **State synchronisation (not just commands).**
   Operator clicks *LiDAR → FAILED* on the dashboard. The dashboard publishes
   `{"lidar":"failed",…}` on `/twin_fault_state`. `SystemMonitorNode` *fuses*
   that override with raw sensor data and publishes the authoritative
   `/system_health` with `twin_mode=FAULTED`. `SafetyStopNode` reads
   `/twin_fault_state` and gates `/cmd_vel`. `RobotResourceNode` clamps battery
   on `battery=clamped`. The dashboard mirrors the resulting red `twin_mode`
   banner — proving the loop closes through the physical side.

3. **Environmental interaction across the twin.**
   An obstacle in Gazebo is sensed by `/scan`. `SafetyStopNode` halts the
   robot, **publishes** the obstacle on `/obstacle_status`, and the dashboard
   turns its "Obstacle" panel red — the same world event now lives on both
   sides of the twin.

---

## 2. The 13 custom nodes (by responsibility)

| Node | Concern | Subscribes | Publishes |
|---|---|---|---|
| `crop_decision_node` | FSM brain, sustainability rule | per-robot state + field arrays | `dispatch_nav_goal`, `irrigate_zone`, `fertilise_zone`, `cmd_vel_nav`, `refill_resources`, `sdg14_intervention`, `supervisor/zone_request` |
| `navigation_executor_node` | Nav2 action-client relay | `dispatch_nav_goal` | `navigation_executor_status` |
| `safety_stop_node` | Multi-sector LiDAR guard, last-line filter | `/scan`, `/cmd_vel_raw`, `/weather_forecast`, `/twin_fault_state` | `/cmd_vel`, `/obstacle_status` |
| `zone_detector_node` | Physical zone verification | TF `map→base_footprint` | `/current_zone`, `/zone_markers` (RViz) |
| `ground_truth_localization` | `map→odom` from Gazebo truth (sim) | gz `/world/.../dynamic_pose/info` | TF `map→odom` |
| `field_sensor_mock_node` | Soil telemetry sim | `/irrigate_zone`, `/fertilise_zone`, `/weather_forecast` | `/field_moisture`, `/field_nutrients`, `/field_growth`, `/field_vulnerability` |
| `weather_adapter_node` | Open-Meteo bridge | (HTTP) | `/weather_forecast` |
| `robot_resource_node` | Battery + fertilizer tank | `odom`, `fertilise_zone`, `refill_resources`, `/twin_fault_state` | `robot_resources` |
| `system_monitor_node` | Hardware watchdog | `/battery_state`, `/scan`, `/imu`, `/twin_fault_state` | `/system_health` |
| `twin_supervisor_node` | Task dispatcher + fault monitor | `/weather_forecast`, `/robot_resources`, `navigation_executor_status`, `supervisor/zone_request` | `/supervisor/zone_assignment`, `/sync_status`, `/system_alerts` |
| `dashboard_node` | Tk GUI (digital entity) | robot telemetry + field arrays | `/weather_forecast`, `/twin_fault_state`, `/generate_report` |
| `sustainability_audit_node` | Independent ledger | `/weather_forecast`, `/fertilise_zone`, `/irrigate_zone`, `/sdg14_intervention`, `/generate_report` | `nexus_farm_report.md` (file) |
| `zone_visualizer_node` | Gazebo coloured tiles | `/field_moisture`, `/field_nutrients`, `/field_vulnerability` | gz `/world/.../create`/`remove` services |

---

## 3. Logical layers

```
┌────────────────────────────────────────────────────────────────────┐
│ Layer 4  Orchestration: TwinSupervisor + SustainabilityAudit       │
├────────────────────────────────────────────────────────────────────┤
│ Layer 3  Intelligence: CropDecision (FSM, sustainability rule)     │
├────────────────────────────────────────────────────────────────────┤
│ Layer 2  Robot I/O: NavigationExecutor, SafetyStop, ZoneDetector,   │
│           RobotResource, SystemMonitor                              │
├────────────────────────────────────────────────────────────────────┤
│ Layer 1  Environment: FieldSensorMock, WeatherAdapter,              │
│           ZoneVisualizer, GroundTruthLocalization                  │
├────────────────────────────────────────────────────────────────────┤
│ Layer 0  Driver stack: Nav2 + SLAM Toolbox + ROS↔Gazebo bridge +   │
│           robot_state_publisher                                     │
└────────────────────────────────────────────────────────────────────┘
```

The dashboard is *orthogonal* — it lives across all four upper layers but
never owns any state of its own; it is purely a mirror + injector.

---

## 4. Command & telemetry pipelines

### 4.1 Velocity command pipeline

```
Nav2  ─► /cmd_vel_nav  ─► [Treatment path]
                             │
                             ▼
                (Spin-in-place actuator when treatment is active)
                             │
                             ▼  mux to /cmd_vel_raw
   /cmd_vel_nav  ─┐
                  ├─► /cmd_vel_raw ──► SafetyStopNode ──► /cmd_vel ──► robot
   /cmd_vel_treat ┘
```

`base.launch.py` points Nav2's `cmd_vel_topic` at `cmd_vel_raw` so that
treatment rotation (published by `crop_decision_node` on `cmd_vel_nav`) and the
nav planner share a single muxed upstream feed. `SafetyStopNode` is the last
gate before `/cmd_vel` reaches the Gazebo bridge.

### 4.2 Treatment pipeline

```
CropDecisionNode  ─► /irrigate_zone    ─► FieldSensorMockNode
                  ─► /fertilise_zone   ─► FieldSensorMockNode + RobotResourceNode + SustainabilityAudit
                  ─► /sdg14_intervention ─► SustainabilityAuditNode (only on HALT_FERTILIZER)
```

All three channels are `String` topics carrying JSON `{"robot":…, "zone":…}`
(optionally with extra fields for interventions). They use
`STATE_QOS = RELIABLE + TRANSIENT_LOCAL depth 10` so late-joining subscribers
get the last value.

### 4.3 Twin supervisor pipeline

```
CropDecisionNode  ─► /supervisor/zone_request ─► TwinSupervisor
                                                    │
                          (fault/weather/battery check)
                                                    ▼
                       /supervisor/zone_assignment ─► CropDecisionNode

       (storm / battery low)  ─► /system_alerts ─► CropDecisionNode (ABORT)
```

### 4.4 Health & fault fan-out

```
/scan /imu /battery_state ─► SystemMonitorNode ─► /system_health ─► DashboardNode
                           (also reacts to /twin_fault_state)

/twin_fault_state ─► SafetyStopNode      (gates cmd_vel, narrows stop dist)
                  ─► RobotResourceNode   (clamps battery on fault)
                  ─► SystemMonitorNode   (overrides raw verdict)

dashboard ─► /twin_fault_state ─┘
```

`SystemMonitorNode` is the **single authoritative publisher** of
`/system_health`. Every other consumer (Dashboard, etc.) reads from it.

---

## 5. Per-robot namespaces

Most application topics are *relative* (no leading `/`) so that, if a future
version runs in `tb3_A/...` and `tb3_B/...` namespaces, both robots coexist on
one ROS graph. The launch file currently launches one of each but the code is
already namespace-safe (`robot_id` parameter is honoured).

Shared topics (used by both the supervisor, the dashboard and the audit) are
*absolute* (`/field_*`, `/scan`, `/system_health`, `/obstacle_status`,
`/twin_fault_state`, `/weather_forecast`, `/supervisor/zone_assignment`,
`/system_alerts`, `/sync_status`, `/generate_report`, `/sdg14_intervention`).

---

## 6. Threading model

- **ROS spin thread per node** — standard `rclpy.spin(node)`.
- **`DashboardNode`** is special: it spins rclpy on a **daemon thread** and
  drives Tk's `mainloop` on the main thread. All Tk widget mutations happen on
  the main thread (in `update_gui_loop` every 100 ms); callbacks only touch a
  guarded `state` dict under `threading.Lock`.
- **`GroundTruthLocalizationNode`** spawns a daemon thread that runs
  `gz topic -e` as a subprocess and parses the stream line-by-line, storing
  the latest `(x, y, yaw)` for the TF timer to read.

---

## 7. Failure isolation patterns

| Risk | Defence |
|---|---|
| Nav2 returns spurious failure | Supervisor de-bounces with `_consecutive_nav_failures >= nav_fault_threshold` (default 3) before declaring fault |
| Robot crashes into wall | `SafetyStopNode` is a defence-in-depth layer between Nav2 and `/cmd_vel`, with 5-sector LiDAR and pre-steering nudge |
| LiDAR cable pulled | `scan_stale_sec` blocks forward motion; `obstacle_status=SCAN_STALE` |
| Dashboard injects invalid weather | Weather scaling is multiplicative; command still flows, robot only slows down |
| Robot can't reach zone (battery too low) | Predictive battery check before accepting assignment; refuses + returns to base |
| CropDecision in wrong zone after Nav2 success | `VERIFYING_ZONE` re-confirms via `current_zone` before scanning |
| Gazebo simulation hitches | All timers are independent; rosbag recording caps at 5 min (`--max-bag-duration 300`) |