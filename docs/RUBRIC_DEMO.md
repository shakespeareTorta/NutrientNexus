# Nutrient Nexus — Rubric Demonstration Guide

How to demonstrate each `rubrics.txt` criterion for the **Option B** digital twin
(Gazebo robot = *physical* entity, Tkinter dashboard = *digital* entity).

> **Option B ceilings** (from the rubric): Bidirectional = **3 (Autobahn)**,
> State Sync = **2 (Cruising)**, Environmental = **2 (Cruising)**.
> This system comfortably exceeds every minimum, so the goal of the demo is to
> *clearly and repeatably show* each data path so the grader can tick the box.

---

## 0. Launch (single command)

```bash
cd ~/nexus-repo && source /opt/ros/jazzy/setup.bash
colcon build --packages-select my_turtlebot3_controller --symlink-install
source install/setup.bash && export TURTLEBOT3_MODEL=burger
ros2 launch my_turtlebot3_controller nexus.launch.py
```

This starts Gazebo, Nav2, ground-truth localization, the 11 application nodes,
the dashboard window, and an automatic `ros2 bag` recording of the key topics
(evidence for the report).

**The two entities**

| Physical (robot side) | Digital (twin side) |
|---|---|
| Gazebo robot, `SafetyStop`, `NavigationExecutor`, `ZoneDetector`, `RobotResource`, `SystemMonitor`, `FieldSensorMock`, `CropDecision` | `Dashboard` (GUI), `TwinSupervisor`, `SustainabilityAudit`, `ZoneVisualizer` |

---

## 1. Bidirectional Communication  →  target **Autobahn (3)**

> *"At least one topic each direction, demonstrated clearly and repeatably."*
> We have several each way.

**Physical → Digital** (robot publishes, dashboard/supervisor consume):
`/robot_resources`, `/current_zone`, `/navigation_executor_status`,
`/system_health`, `/obstacle_status`, `/field_moisture|_nutrients|_vulnerability`.

**Digital → Physical** (dashboard/supervisor publish, robot side consumes):
`/weather_forecast`, `/twin_fault_state`, `/supervisor/zone_assignment`,
`/system_alerts`, `/generate_report`.

**Demo steps**
1. Show the dashboard updating live as the robot drives (battery bar drops,
   Zone/Nav labels change) → **Physical → Digital**.
2. Click a **Weather** button (e.g. *Rainy*) → robot visibly slows in Gazebo →
   **Digital → Physical**.
3. Prove it on the wire (second terminal):
   ```bash
   ros2 topic echo /robot_resources      # robot → twin, streaming
   ros2 topic echo /weather_forecast     # twin → robot, on button press
   ros2 topic hz /robot_resources        # show consistent ~1 Hz, no dropouts
   ```

---

## 2. Synchronization of States  →  target **Cruising (2)** (need ≥1 non-motion state mirrored)

> *"pose + one simple status … at least one non-motion state mirrored."*
> We mirror several internal states, including ones that change behaviour.

Mirrored on the dashboard from the physical side:
- **Battery %** and **Fertilizer %** (`/robot_resources`).
- **Twin mode** banner NORMAL / DEGRADED / FAULTED (`/system_health`).
- **LiDAR / IMU / Battery health** labels (`/system_health`).
- **Current zone** + **navigation status** (motion state).

**Demo: full fault round-trip (digital → physical → digital)**
1. On the dashboard click **LiDAR → DEGRADED/FAILED** (or **Motor → STALLED**).
2. `Dashboard` publishes `/twin_fault_state` → `SystemMonitor` fuses it →
   re-publishes `/system_health` with the fault and `twin_mode: FAULTED` →
   `SafetyStop` freezes the robot (motor) or blocks forward motion (lidar).
3. The dashboard banner turns red/orange and the health label flips — the
   injected state has propagated through the physical side and back.
4. Click **Clear All Faults** → everything returns to NORMAL.
   ```bash
   ros2 topic echo /system_health        # watch twin_mode + statuses change
   ros2 topic echo /twin_fault_state
   ```

---

## 3. Environmental & Object Interaction  →  target **Cruising (2)**

> *"avoid obstacle OR push/track object, mirroring partial/delayed is fine."*
> We show obstacle avoidance mirrored to the digital entity, plus a weather and a
> treatment loop.

**Primary: obstacle avoidance, mirrored.**
1. While the robot is driving, drop an obstacle (a box / another model) in front
   of it in Gazebo (Insert → simple shape), or steer it toward a wall.
2. `SafetyStop` sees it on `/scan`, stops/nudges the robot, and publishes
   `/obstacle_status`.
3. The dashboard **Environment / Obstacle** panel flips from green
   *"No obstacle"* to red *"OBSTACLE: FRONT @ 0.28m"* — the environment event did
   **not** stay local to the robot.
   ```bash
   ros2 topic echo /obstacle_status
   ```

**Secondary (extra credit): environment ⇄ twin loops**
- **Weather:** dashboard *Storm* → `FieldSensorMock` leaches nutrients faster and
  raises runoff vulnerability, `SafetyStop` slows the robot, `TwinSupervisor`
  broadcasts a storm abort → robot returns to base.
- **Treatment:** robot reaches a low zone and fertilises/irrigates →
  `FieldSensorMock` restores that zone's `/field_nutrients|_moisture` → the
  dashboard **Field Zone Status** row and the **Gazebo tile colour** both flip to
  HEALTHY/green. The robot's action on the field propagates back to the twin.

---

## 4. One clean 3–4 minute demo script

1. Launch; let the robot patrol. Point out the dashboard mirroring battery, zone,
   nav, and the live zone table — **Req 1 (P→D) + Req 2**.
2. Click **Rainy**, then **Storm**; robot slows then returns to base — **Req 1
   (D→P) + Req 3 (weather)**.
3. Drop a box in front of the robot; dashboard shows the obstacle — **Req 3
   (obstacle, mirrored)**.
4. Inject a **LiDAR/Motor fault**; banner → FAULTED, robot stops; **Clear** to
   recover — **Req 2 (internal state round-trip)**.
5. Let it treat a zone; watch the tile + dashboard row turn green — **Req 3
   (action → environment → twin)**.
6. Click **Generate Sustainability Report**; open `nexus_farm_report.md` — closes
   the loop. The auto-recorded rosbag in `/tmp/nexus_recording_*` is your
   evidence artifact.

> Tip: keep one terminal running `ros2 topic echo /obstacle_status` (or
> `/system_health`) on screen during the demo so the grader sees the data on the
> wire at the same time as the GUI reacts.
