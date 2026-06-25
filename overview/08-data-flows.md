# Data Flows (Option-B Demos)

This document walks through the three Option-B demonstration scenarios
exactly as the dashboard docs prescribe them. Each section names the
telemetry streams and overrides involved, in order.

> Open two terminals alongside the dashboard so you can `ros2 topic echo`
> the relevant streams while clicking buttons.

```sh
ros2 topic echo /robot_resources
ros2 topic echo /system_health
ros2 topic echo /weather_forecast
ros2 topic echo /twin_fault_state
ros2 topic echo /obstacle_status
```

---

## 1. Req 1 — Bidirectional pub/sub

**Claim:** the system has real P→D *and* D→P streams, not just commands.

### Side 1 (P→D): robot resources stream

```
RobotResourceNode ──► robot_resources (String JSON, 1 Hz)
                       │
                       ▼
                  DashboardNode.resource_cb
                       │
                       ▼
                  Battery + Fertilizer progress bars (refresh every 100 ms)
```

Open `ros2 topic echo /robot_resources` in a terminal. Drive the robot
around (or wait for an autonomous task to deplete the battery). Watch the
JSON counter change and the dashboard bars shrink.

### Side 2 (D→P): weather injection

```
Operator clicks "Rainy"
       │
       ▼
DashboardNode.weather_pub.publish("rainy")
       │
       ├─► /weather_forecast
       │     │
       │     ▼
       │     FieldSensorMockNode.weather_cb  (drives moisture & nutrient dynamics)
       │     SafetyStopNode._weather_cb      (multiplies linear.x by 0.6)
       │     SustainabilityAuditNode.weather_cb (stamps future log entries)
       │
       └─► (no further effect in the dashboard; the publish already left
            the digital side and is propagating downstream)
```

Open `ros2 topic echo /weather_forecast` in a terminal. Click the four
weather buttons in turn. You should see one publish per click. Now
watch the SafetyStop log line slow down `linear.x` proportionally when
rainy/storm is active, and watch the field simulator's moisture and
nutrient curves change shape.

### Why this is *real* bidirectional

Both streams are running concurrently, both update the same node-graph
state, and neither side polls. The system passes the "two-way flow"
test the assignment asks for.

---

## 2. Req 2 — State synchronisation (not just commands)

**Claim:** the dashboard can change the *state* of the physical robot,
not just send it commands.

### Step 2.1 — Inject a LiDAR fault

```
Operator clicks "LiDAR"  (cycle OK → DEGRADED → FAILED)
       │
       ▼
DashboardNode._publish_faults()
       │
       ▼
/twin_fault_state = {"lidar": "failed", "motor": "ok", "battery": "normal", "active": true}
       │
       ├─► SystemMonitorNode._fault_callback     → overrides verdict
       │                                            /system_health.twin_mode = FAULTED
       │                                            /system_health.lidar.status = FAILED
       │     │
       │     ▼
       │   DashboardNode.health_cb                → TWIN MODE banner turns RED
       │
       ├─► SafetyStopNode._fault_callback        → /cmd_vel forward blocked
       │     │
       │     ▼
       │   SafetyStopNode._publish_obstacle      → /obstacle_status=LIDAR_FAULT
       │
       └─► RobotResourceNode.fault_callback      (no-op for lidar fault)
```

Open `ros2 topic echo /system_health` and watch `twin_mode` flip to
`FAULTED`. The dashboard's top banner turns red. The robot stops
moving forward (rotation in place still works). The Obstacle panel
shows `OBSTACLE: LIDAR_FAULT`.

### Step 2.2 — Inject a battery clamp (hidden button)

The battery-fault button is commented out in the dashboard UI (see
`DashboardNode._build_faults`) but the toggle code is present. To
exercise it, publish the message manually:

```sh
ros2 topic pub --once /twin_fault_state std_msgs/String \
    '{data: "{\"lidar\":\"ok\",\"motor\":\"ok\",\"battery\":\"clamped\",\"active\":true}"}'
```

What happens:

```
/twin_fault_state → SystemMonitorNode   → /system_health.twin_mode = FAULTED
                  → RobotResourceNode   → battery clamped to 10%, drain suspended
                  → DashboardNode       → mirror turns red, twin_mode=FAULTED
                  → CropDecisionNode    → resource guard fires on next tick,
                                           cancels timers, dispatches to base
                                           → /refill_resources on arrival
```

Open `ros2 topic echo /robot_resources`. Watch `battery` jump from
whatever it was to exactly 10.0 and stay there. The brain then
dispatches to base; `navigation_executor_status` will go through
`NAVIGATING` → `SUCCEEDED_AT_POSE` and `current_zone` will show
`base_station`. On arrival the refill runs — fertilizer back to 100,
battery stays at 10 (clamped).

### Why this proves "state sync"

The dashboard *changed* the battery state of the robot — not by sending
a velocity command, but by injecting a state override that propagates
through every consumer. The change is visible on every subsystem's
output topic, and the robot autonomously reacts to it (returns to base).
That's synchronisation, not command-and-forget.

---

## 3. Req 3 — Environmental interaction across the twin

**Claim:** a real Gazebo-world event shows up on the digital side
*and* the digital-side reaction (the safety gate) closes the loop.

### Drive the robot into a wall

```
Gazebo wall ──► /scan (LaserScan) ──► SafetyStopNode.scan_callback
                                         │
                                         ▼
                                    d_front shrinks below 0.22 m
                                         │
                                         ▼
                                    SafetyStopNode.cmd_callback
                                    forward → zero, allow rotation
                                    /cmd_vel = Twist(0, 0, 0, 0, 0, 0)
                                         │
                                         ├─► /cmd_vel → robot (halted)
                                         │
                                         └─► /obstacle_status = {
                                                "blocked": true,
                                                "distance": 0.18,
                                                "sector": "FRONT"
                                             }
                                                │
                                                ▼
                                            DashboardNode.obstacle_cb
                                                │
                                                ▼
                                            Obstacle panel turns RED
```

Open `ros2 topic echo /obstacle_status` and watch the JSON toggle
between `CLEAR` and `{"blocked":true,"distance":…,"sector":"FRONT"}`.
The dashboard's Obstacle panel updates every 100 ms — green when
clear, red when blocked, with the sector and distance shown.

### Try the pre-steering nudge

Approach a wall at an angle (Nav2 will usually approach head-on, but
the world geometry can put the obstacle in a diagonal sector first).
Watch the SafetyStop logs for "Pre-steer nudge LEFT/RIGHT" lines and
the corresponding `sector=FRONT_LEFT|FRONT_RIGHT` in `/obstacle_status`.

### Why this is "across the twin"

The world event (an obstacle) originates in the physical side; the
physical-side safety node enforces the safety behaviour; the dashboard
mirrors what the safety node saw. The two entities agree on the world
state without one polling the other.

---

## 4. Composite demo — the full Option-B loop

In one 60-second run:

1. Let the robot patrol autonomously (no overrides). Watch the
   CropDecisionNode log line: it should report zone visits, decide
   actions, and either fertilise / irrigate / refuse.
2. While patrolling, click **Storm** on the dashboard. Within a tick,
   the supervisor broadcasts `ABORT/STORM_EMERGENCY`. The brain drops
   the queue and dispatches to base. The robot returns.
3. While the robot is on its way back, click **LiDAR → DEGRADED** on
   the dashboard. The robot slows (linear.x × 0.5). The dashboard
   `twin_mode` shows `DEGRADED`.
4. Inject the battery clamp as in §2.2. The robot's battery drops to
   10 %; the brain diverts to base; the dashboard banner turns red.
5. Once the robot is at base, click **Clear All Faults** and **Sunny**.
   The robot resumes patrolling.
6. Throughout, `ros2 topic echo /sync_status` shows the supervisor's
   snapshot every second (robot state, weather, queue length).
7. Click **Generate Sustainability Report** at any time; the audit node
   writes `nexus_farm_report.md` to the working directory.

The full bidirectional twin loop has now been exercised end to end.