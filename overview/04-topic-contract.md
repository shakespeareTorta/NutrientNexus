# Topic Contract

Every topic the system uses, in alphabetical order. Pay attention to the
**QoS** column — most "state" topics use `STATE_QOS` (`RELIABLE +
TRANSIENT_LOCAL, depth 10`) so late-joining subscribers get the last value.

> Topics with a leading `/` are shared / global; others are per-robot (namespace-safe).

---

## 1. Quick index

| Topic | Type | Direction | QoS | Owner → consumers |
|---|---|---|---|---|
| `/battery_state` | `sensor_msgs/BatteryState` | (real only) | BEST_EFFORT | TB3 → SystemMonitor |
| `/clock` | `rosgraph_msgs/Clock` | Gazebo → ROS | (bridge) | Gazebo → all sim nodes |
| `/cmd_vel` | `geometry_msgs/Twist` | out | default | SafetyStop → bridge → robot |
| `/cmd_vel_raw` | `geometry_msgs/Twist` | internal | default | Nav2 + treatment → SafetyStop |
| `/cmd_vel_nav` | `geometry_msgs/Twist` | out (per-robot) | default | Nav2 + treatment publish |
| `/current_zone` | `std_msgs/String` | out (per-robot) | STATE_QOS | ZoneDetector → CropDecision, Dashboard |
| `/dispatch_nav_goal` | `geometry_msgs/PoseStamped` | in (per-robot) | default | CropDecision → NavigationExecutor |
| `/field_growth` | `std_msgs/Float32MultiArray` | global | STATE_QOS | FieldSensorMock → (log) |
| `/field_moisture` | `std_msgs/Float32MultiArray` | global | STATE_QOS | FieldSensorMock → CropDecision, Dashboard, ZoneVisualizer |
| `/field_nutrients` | `std_msgs/Float32MultiArray` | global | STATE_QOS | FieldSensorMock → CropDecision, Dashboard, ZoneVisualizer |
| `/field_vulnerability` | `std_msgs/Float32MultiArray` | global | STATE_QOS | FieldSensorMock → CropDecision, Dashboard, ZoneVisualizer |
| `/fertilise_zone` | `std_msgs/String` (JSON) | out (per-robot) | STATE_QOS | CropDecision → RobotResource, FieldSensorMock, Audit |
| `/generate_report` | `std_msgs/String` | in | default | Dashboard → Audit |
| `/imu` | `sensor_msgs/Imu` | (real only) | BEST_EFFORT | TB3 → SystemMonitor |
| `/irrigate_zone` | `std_msgs/String` (JSON) | out (per-robot) | STATE_QOS | CropDecision → FieldSensorMock, Audit |
| `/joint_states` | `sensor_msgs/JointState` | bridge | (bridge) | Gazebo → robot_state_publisher |
| `/navigation_executor_status` | `std_msgs/String` | out (per-robot) | STATE_QOS | NavigationExecutor → CropDecision, Supervisor, Dashboard |
| `/obstacle_status` | `std_msgs/String` (JSON) | out | STATE_QOS | SafetyStop → Dashboard |
| `/odom` | `nav_msgs/Odometry` | bridge | (bridge) | Gazebo → RobotResource, CropDecision, GroundTruth |
| `/refill_resources` | `std_msgs/String` | out (per-robot) | STATE_QOS | CropDecision → RobotResource |
| `/robot_resources` | `std_msgs/String` (JSON) | out (per-robot) | STATE_QOS | RobotResource → CropDecision, Supervisor, Dashboard |
| `/scan` | `sensor_msgs/LaserScan` | bridge | (bridge) | Gazebo → SafetyStop, SystemMonitor |
| `/sdg14_intervention` | `std_msgs/String` (JSON) | out (per-robot) | STATE_QOS | CropDecision → Audit |
| `/supervisor/zone_assignment` | `std_msgs/String` (JSON) | global | STATE_QOS | Supervisor → CropDecision |
| `/supervisor/zone_request` | `std_msgs/String` (JSON) | out (per-robot) | default | CropDecision → Supervisor |
| `/sync_status` | `std_msgs/String` (JSON) | global | STATE_QOS | Supervisor → (audit / log) |
| `/system_alerts` | `std_msgs/String` (JSON) | global | STATE_QOS | Supervisor → CropDecision |
| `/system_health` | `std_msgs/String` (JSON) | out | STATE_QOS | SystemMonitor → Dashboard |
| `/tf` | `tf2_msgs/TFMessage` | bridge | (bridge) | Gazebo → TF tree |
| `/twin_fault_state` | `std_msgs/String` (JSON) | out | STATE_QOS | Dashboard → SystemMonitor, SafetyStop, RobotResource |
| `/weather_forecast` | `std_msgs/String` | out | STATE_QOS | WeatherAdapter / Dashboard / Supervisor → FieldSensorMock, SafetyStop, Audit |
| `/zone_markers` | `visualization_msgs/MarkerArray` | out (per-robot) | default | ZoneDetector → RViz |

---

## 2. JSON schemas

### `/twin_fault_state` (Dashboard → physical)
```json
{
  "lidar":   "ok|degraded|failed",
  "motor":   "ok|stalled",
  "battery": "normal|clamped",
  "active":  true
}
```
* `motor=stalled` → SafetyStop publishes zero twist (`MOTOR_FAULT`).
* `lidar=failed` → forward blocked, rotation allowed (`LIDAR_FAULT`).
* `lidar=degraded` → forward speed halved, stop distance × 1.5.
* `battery=clamped` → RobotResource clamps battery to 10 % and suspends drain;
  SystemMonitor sets `twin_mode=FAULTED`; CropDecision will trip its resource
  guard and head home.

### `/system_health` (SystemMonitor → Dashboard)
```json
{
  "battery":  {"status": "OK|WARNING|CRITICAL|NO_DATA", "voltage": …, "percent": …, "present": …},
  "lidar":    {"status": "OK|STALE|HIGH_DROPOUT|ALL_INVALID|EMPTY",
               "total_rays": …, "valid_rays": …, "dropout_pct": …, "age_sec": …},
  "imu":      {"status": "OK|STALE|NO_DATA", "accel_ms2": …, "gyro_rps": …},
  "faults":   {"lidar": "ok|degraded|failed", "motor": "ok|stalled", "battery": "normal|clamped"},
  "motor":    {"status": "OK|STALLED"},
  "twin_mode": "NORMAL|DEGRADED|FAULTED"
}
```

### `/obstacle_status` (SafetyStop → Dashboard)
```json
{"blocked": true, "distance": 0.18, "sector": "FRONT|FRONT_LEFT|FRONT_RIGHT|MOTOR_FAULT|LIDAR_FAULT|SCAN_STALE|CLEAR"}
```
`distance = -1.0` when not finite (e.g. SCAN_STALE).

### `/robot_resources` (RobotResource → others)
```json
{"battery": 78.3, "fertilizer": 60.0}
```

### `/supervisor/zone_assignment` and `/supervisor/zone_request`
```json
{"robot": "A", "zone": "zone_0"}
{"robot": "A", "zone": "BASE"}
```

### `/system_alerts`
```json
{"action": "ABORT", "reason": "STORM_EMERGENCY|PHYSICAL_ROBOT_FAULT"}
```

### `/sdg14_intervention`
```json
{"robot": "A", "zone": "zone_2", "action": "HALT_FERTILIZER", "reason": "High runoff risk"}
```

### `/irrigate_zone` and `/fertilise_zone`
```json
{"robot": "A", "zone": "zone_2"}
```

### `/navigation_executor_status` (text)
One of: `IDLE`, `NAVIGATING`, `SUCCEEDED_AT_POSE`, `FAILED_NAVIGATION`,
`ABORTED_NAVIGATION`, `CANCELED_NAVIGATION`, `REJECTED`,
`IDLE_SERVER_UNAVAILABLE`. The supervisor recognises any state with
`'fault'` in its name as a failure for streak counting.

### `/current_zone` (text)
A key of `zones.yaml` (`base_station`, `zone_0`, …) or `no_zone`.

### `/weather_forecast` (text)
`sunny` | `rainy` | `overcast` | `storm`.

### `/sync_status`
```json
{"robot": {"status": "…", "battery": …, "zone": "…", "faulted": false},
 "weather": "sunny",
 "tasks_remaining": 4}
```

### `/field_moisture`, `/field_nutrients`, `/field_growth`, `/field_vulnerability`
`std_msgs/Float32MultiArray` with one entry per zone, **indexed by
alphabetical zone id, base_station excluded** (so for four zones the array
has 4 entries in zone_0, zone_1, zone_2, zone_3 order). The layout label is
`zones`.

---

## 3. `STATE_QOS` (the shared profile)

```python
# src/my_turtlebot3_controller/my_turtlebot3_controller/qos.py
STATE_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
```

Used by every topic where late subscribers must see the last value
(battery, weather, fault state, telemetry arrays, zone, etc.). Without
TRANSIENT_LOCAL a new dashboard would only see deltas after the first
publish.

---

## 4. Sensor topics and their QoS

`/scan` and (real-robot only) `/imu`, `/battery_state` are subscribed by
`SystemMonitorNode` and `SafetyStopNode` with `BEST_EFFORT` to match the
rate of the sensor stream and to be robust to small drops.