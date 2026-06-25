# Safety & Faults

This document describes how the system defends the robot from the world
(`SafetyStopNode`) and how the digital side of the twin can inject
hardware faults that cascade through the system. Both pathways converge
on a single state view (`SystemMonitorNode`).

---

## 1. The defence layers (depth-in-depth)

```
Operator intent     ──►  Dashboard  ──►  /twin_fault_state  ──┐
                                                               │
Nav2 plan           ──►  /cmd_vel_nav  ─┐                       │
                                      ├──► /cmd_vel_raw ──► SafetyStop ──► /cmd_vel ──► robot
Treatment actuator  ──►  /cmd_vel_nav  ─┘                       │
                                                               │
World (walls, etc.) ──►  /scan  ─────────────────────────────────┘
```

The `SafetyStopNode` is the **last gate** before the bridge, so every
velocity command (Nav2, treatment, future teleop) is filtered through it.

---

## 2. `SafetyStopNode` — five-sector guard

### Sector definitions (half-widths from forward axis)
| Sector | Range | Trigger | Outcome |
|---|---|---|---|
| FRONT | ±15° | `d_front < stop_distance` (0.22 m) | zero forward, allow rotation |
| FRONT_LEFT | +15°..+70° | `d_front_left < stop_distance × 1.2` | nudge right (`+angular.z`) |
| FRONT_RIGHT | −70°..−15° | `d_front_right < stop_distance × 1.2` | nudge left (`−angular.z`) |
| LEFT | +70°..+130° | informational | (used for collision awareness, not gating) |
| RIGHT | −130°..−70° | informational | (used for collision awareness, not gating) |

### Decision order (highest priority first)
1. **Motor fault = stalled** → publish zero twist, `obstacle=MOTOR_FAULT`.
2. **Weather scaling** on `linear.x`:
   * `rainy` → `linear.x *= 0.6`
   * `storm` → `linear.x *= 0.4`
3. **LiDAR fault = failed** → forward blocked (rotation allowed),
   `obstacle=LIDAR_FAULT`.
4. **LiDAR fault = degraded** → `linear.x *= 0.5`, `stop_distance *= 1.5`.
5. **Scan stale** (`age > scan_stale_sec`) or never seen → forward blocked
   (rotation allowed), `obstacle=SCAN_STALE`.
6. **Front blocked + forward requested** → zero forward, allow rotation,
   `obstacle=FRONT`.
7. **Pre-steering nudge** when a diagonal sector is closing in but FRONT
   is still clear (and the upstream command isn't already turning):
   * `d_front_left < nudge_threshold` → `angular.z = −nudge_turn_speed × nudge_factor` (`obstacle=FRONT_LEFT`).
   * `d_front_right < nudge_threshold` → `angular.z = +nudge_turn_speed × nudge_factor` (`obstacle=FRONT_RIGHT`).
8. Otherwise → pass-through, `obstacle=CLEAR`.

### Narrow-object detection

`sector_min` would miss a thin obstacle (e.g. a chair leg) that only
hits 1-3 LiDAR rays because the sector's minimum is dominated by the
**minimum** range over many rays. `narrow_object_in_sector` checks if
**any** ray in `[-side, +side]` is closer than `narrow_obj_dist`
(default 0.18 m) and pulls `d_front` down to that threshold so the
front-block check fires.

### `obstacle_status` deduplication

Payloads that match the last one published are dropped, so the topic
doesn't churn during long stretches of `CLEAR` or sustained `FRONT`.

---

## 3. Fault injection (digital → physical)

The dashboard exposes three fault toggles. Each publishes a JSON state
on `/twin_fault_state` (RELIABLE + TRANSIENT_LOCAL) with the shape:
```json
{"lidar": "ok|degraded|failed", "motor": "ok|stalled", "battery": "normal|clamped", "active": bool}
```

### Cascade map
| Dashboard action | First effect | Cascades to |
|---|---|---|
| **LiDAR → DEGRADED** | `linear.x *= 0.5`, `stop_distance *= 1.5` | `/obstacle_status` continues to publish; `twin_mode=DEGRADED` |
| **LiDAR → FAILED** | forward motion blocked, rotation allowed | `twin_mode=FAULTED`; dashboard banner turns red |
| **Motor → STALLED** | zero twist from SafetyStop, `obstacle=MOTOR_FAULT` | `twin_mode=FAULTED`; the robot halts in place |
| **Battery → CLAMPED** (hidden button) | `RobotResource` clamps to 10 % and suspends drain | `CropDecision` resource guard fires → `RETURNING_TO_BASE`; `twin_mode=FAULTED`; supervisor also pauses if battery drops below 15 % |

The button is shared across all consumers: `SystemMonitorNode` reads it
for the health verdict, `SafetyStopNode` reads it for motion gating,
`RobotResourceNode` reads it for the battery clamp. There is **no**
local copy of the fault state in any of them; the latest publication on
`/twin_fault_state` is the source of truth.

---

## 4. The watchdog (`SystemMonitorNode`)

Subscribes (BEST_EFFORT) to `/scan`, `/imu`, `/battery_state` and
(RELIABLE) to `/twin_fault_state`. Publishes `/system_health` at
`check_hz` (default 2 Hz). It is the **only** publisher of
`/system_health` — every other consumer reads from it.

### Per-subsystem classification

**Battery**
* NO_DATA (sim): no `/battery_state` yet.
* CRITICAL if `voltage ≤ crit_voltage` (10.8 V) or `percent ≤ crit_percent` (20 %).
* WARNING if `voltage ≤ warn_voltage` (11.5 V) or `percent ≤ warn_percent` (35 %).
* OK otherwise.
* Also flags `present=false` as a separate "battery not present" alarm.

**LiDAR**
* STALE if no scan ever OR `age > scan_stale_sec` (2 s).
* EMPTY if the scan carries 0 rays.
* ALL_INVALID if every ray is non-finite or out-of-range.
* HIGH_DROPOUT if ≥ `dropout_pct` (40 %) of rays are bad.
* OK otherwise.

**IMU**
* NO_DATA if no message ever (sim path) or stale > `imu_stale_sec` (2 s).
* Logs HIGH_ACCEL if `|accel| > accel_warn_g × g` (2.5 g).
* Logs HIGH_GYRO if `|gyro| > gyro_warn_rps` (5.0 rad/s).
* Status always OK when data is present and fresh.

### Injected-fault override
The raw verdict is overridden by the latest fault:
* `lidar=failed` → status forced to `FAILED` regardless of scan content.
* `lidar=degraded` and raw `OK` → forced to `DEGRADED`.
* `motor=stalled` → `motor = {"status":"STALLED"}`.
* `motor=ok` → `motor = {"status":"OK"}`.

### Derived `twin_mode`
| Condition | twin_mode |
|---|---|
| motor=stalled OR lidar=failed OR battery=clamped OR battery raw=CRITICAL | FAULTED |
| lidar=degraded OR lidar raw in {STALE, HIGH_DROPOUT, ALL_INVALID} OR battery raw=WARNING | DEGRADED |
| otherwise | NORMAL |

---

## 5. De-bouncing (the supervisor's role)

A single Nav2 abort is a normal, recoverable event. To stop transient
abort noise from triggering global abort storms, the supervisor only
declares a "physical robot fault" after `nav_fault_threshold` (default 3)
**consecutive** navigation failures with no successful arrival between
them. One `SUCCEEDED_AT_POSE` resets the streak to zero.

`/system_alerts` is the broadcast channel for both storm-induced aborts
and physical-fault aborts. `CropDecisionNode.alerts_callback` reads it
in `WAITING_FOR_ASSIGNMENT` and any other phase, clears the sub-target
queue and dispatches to base.

---

## 6. Resource-guard safety net

`CropDecisionNode.state_machine_tick` runs a resource check at the top
of every 1 Hz tick. If `battery ≤ low_battery_threshold` (15 %) or
`fertilizer ≤ low_fertilizer_threshold` (10 %):

1. `_cancel_and_destroy` all in-flight timers (scan / actuation / cooldown).
2. Publish a zero twist on `cmd_vel_nav` so the actuator stops spinning.
3. Clear the sub-target queue.
4. Dispatch to base.

This guarantees the robot never gets stranded mid-zone, regardless of
which phase it was in.

---

## 7. What still needs the operator's eye

* **Slow scan failure** — `slam_toolbox` is replaced by ground-truth TF,
  so this is a sim-only consideration. On a real robot, SLAM quality
  would matter.
* **Long paths to corners** — Nav2 was historically bound to a 0.2 s
  transform tolerance; the params file raises this to 1.0 s with an
  explicit comment that SLAM's `map→odom` was stalling to ~1.6 s during
  map rebuilds.
* **Map rebuilds** — irrelevant in sim, would matter on real hardware.