# NutrientNexus — Option B Status & To-Do (Team Brief)

This document summarizes where our digital-twin implementation stands against the
**Option B** requirements, and what still needs to be done. It is the result of comparing
the assignment's "Option B" specification against the current code in the repo.

---

## What Option B requires

We do **not** need a physical robot. Option B asks for **two distinct entities** plus three
demonstrable digital-twin properties.

**Entity mapping we are using:**
- **Physical entity (stand-in):** the **Gazebo TurtleBot3 and its ROS nodes** — the source
  of truth for `/scan`, `/odom`, battery/fertilizer, localization zone, nav status, and
  field telemetry.
- **Digital entity:** the **Dashboard** (`DashboardNode`) — mirrors/visualizes the physical
  side and sends control + injection commands back to it.

**The single-robot launch file `nexus.launch.py` is our primary Option B vehicle.**
`nexus_dual.launch.py` stays functional but is not the twin pair.

**The three properties we must prove (and label in the demo):**
1. **Bidirectional pub/sub** — physical→digital (telemetry) AND digital→physical
   (weather injection, fault injection, optional override).
2. **State synchronization (not just commands)** — battery, sensor status
   (working/failed/degraded), operating mode. A change on one side must reflect on the other
   in (near-)real-time and must affect behavior or presentation.
3. **Environmental interaction** — obstacle / world changes must **propagate to both sides**,
   not stay local to the robot.

---

## ✅ What is already done

**Requirement 1 — Bidirectional pub/sub (partially in place):**
- Physical → Digital: `robot_resource_node` → `/robot_resources`, `zone_detector_node` →
  `/current_zone`, `navigation_executor_node` → `/navigation_executor_status`; the dashboard
  subscribes to all of these.
- Digital → Physical: the dashboard publishes `/weather_forecast` → `FieldSensorMockNode`,
  `TwinSupervisorNode`, and `SustainabilityAuditNode` subscribe. (The minimum two-way
  requirement is technically met.)

**Requirement 2 — State synchronization (basic):**
- Battery and fertilizer levels: `robot_resource_node` computes them (battery from `/odom`
  distance, fertilizer from spray events) and the dashboard mirrors them as progress bars
  that turn red below 30% / 20%.
- Weather "mode" propagation: injecting weather from the dashboard changes the field sensor
  behavior, and a storm triggers an ABORT from the supervisor.

**Requirement 3 — Environmental interaction (partial):**
- Obstacle detection & avoidance: `SafetyStopNode` reads `/scan`, filters `/cmd_vel_nav`,
  and blocks forward motion (while allowing rotation) when an obstacle is in the front arc.
- Weather injection → `FieldSensorMockNode` changes moisture/nutrients (a world change that
  affects sensing).

**Infrastructure:** Gazebo + Nav2 + SLAM bring-up (`base.launch.py`), ROS↔GZ bridge, rosbag
auto-recording, RViz, and zone bounding-box markers are all working.

---

## ❌ What still needs to be done (prioritized)

| # | Task | Why it's needed |
|---|------|-----------------|
| 1 | **Push obstacle/scan state to the dashboard** (digital entity must "see" the obstacle: publish a min-front-distance / blocked flag → dashboard subscribes and displays it) | Requirement 3's core rule: an environmental event must **propagate to both sides**. Right now the obstacle reaction is local to the robot — our weakest rubric point |
| 2 | **Add fault / sensor-failure injection** (dashboard button → a node degrades/fails the sensor → behavior changes and the dashboard shows it) | The headline Option B example. Cleanly satisfies "sensor status working/failed/degraded" + "mode" under Requirement 2. Currently absent |
| 3 | **Fix the payload mismatch:** `CropDecisionNode` publishes JSON to `/fertilise_zone`, but `FieldSensorMock` and `Audit` expect a plain zone string | Silent failure → fertilization replenishment and audit logging don't run → the "treatment → field state change → mirrored on dashboard" loop is broken (Requirement 2 evidence collapses) |
| 4 | **Actually implement the SDG-14 intervention pipeline:** `CropDecisionNode` declares `/sdg14_intervention` but never publishes; the `DECIDING` step is empty | The audit ledger stays empty → our main sustainability demo produces nothing. `DECIDING` must read moisture/nutrients/weather/runoff-risk and decide Irrigate / Fertilize / Halt |
| 5 | **Wire field telemetry into decisions:** `moisture_callback`/`nutrients_callback` are currently `pass`; `/field_growth` is never consumed | Decisions are hardcoded, not data-driven. Requirement 2 (state must affect behavior) needs the telemetry → decision link |
| 6 | **Set `use_sim_time: True` on all custom nodes in `nexus.launch.py`** (the dual launch already does this) | Sim-time mismatch breaks TF lookups (especially `zone_detector`'s `map → base_footprint`) and timers |
| 7 | **Connect or remove the operator-override loop:** the dashboard subscribes to `/operator_override_request` and publishes `/operator_override_response`, but nothing publishes the request or consumes the response | If connected, this becomes a strong **digital→physical control** (human-in-the-loop) demo: the robot asks the operator before a risky spray. Otherwise it's dead code |
| 8 | **Lock in the twin pair:** use single-robot `nexus.launch.py` (Gazebo = physical, Dashboard = digital); keep the dual launch as an extra | In the dual launch, Robot A and B don't mirror each other — they patrol separate zones, so they don't fit the "physical vs digital mirror" story and would confuse the demo |
| 9 | **Dashboard decision:** keep Tkinter (an allowed "control panel") and fix the README; or move to a web dashboard (matches the Option B example, easier for live `/scan` + metrics) | The README says "Flask" but the code is Tkinter. For now we keep Tkinter and fix the docs; a web dashboard is an optional upgrade |
| 10 | **Dead twin code:** `TwinSafetyNode` (dual-scan fusion) and `OdomToGazeboPoseNode` are written but not in any launch file — they were for Option A (real + sim) | Decide: leave them with a clear "Option A, not used in Option B" note. Obstacle propagation for Option B is handled by task 1, not by `TwinSafetyNode` |

---

## How this maps to the required demo evidence

The assignment requires three labeled pieces of evidence in the demo:
- **Bidirectional pub/sub** (topic list + echo/logs) → strengthened by tasks 1, 2, 7.
- **State synchronization** (one side changes → the other mirrors) → tasks 2, 3, 5.
- **Environmental interaction** (obstacle/world change → mirrored response) → tasks 1, 2.

**Highest-leverage first:** task 3 (payload fix) and task 1 (scan → dashboard) — one repairs
a broken loop, the other closes our weakest rubric gap.
