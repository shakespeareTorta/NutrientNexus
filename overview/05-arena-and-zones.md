# Arena & Zones

The simulated world is a custom Gazebo SDF (gz-sim Harmonic) with a
rectangular room and a base station in the middle. Four crop zones sit
near the four corners, each as a bounding box the robot must enter to
treat.

---

## 1. The Gazebo world

* **File:** `src/my_tb3_world/worlds/new_world.world` (56 KB).
* **Format:** SDF for gz-sim Harmonic.
* **World name:** `default`.
* **Composition:** a single `ground_plane` + ~40 static boxes (`box`, `box_1`,
  `box_2`, …, `box_8_5_5`) that form the room walls and internal panels.
* **Outer walls:** `X ∈ [-1.02, 2.63]`, `Y ∈ [-3.86, 1.00]` (a tall
  rectangle).
* **Default robot spawn (from `nexus.launch.py`):** `x=0.6`, `y=-1.6`,
  `yaw=1.5708` (90°, facing the room interior).

The world is installed to `share/my_tb3_world/worlds/new_world.world` by the
`my_tb3_world` package's `CMakeLists.txt`. If that package is not built /
sourced, `base.launch.py` falls back to `empty.sdf` with a loud warning.

---

## 2. The four zones

From `src/my_turtlebot3_controller/config/zones.yaml`:

| ID | Label | Target (x, y, θ°) | Bounding box (x, y) | Baseline moisture | Baseline nutrients | Runoff risk |
|---|---|---|---|---|---|---|
| `base_station` | Base | (−0.1, 0.0, 0°) | x ∈ [−0.40, 0.20], y ∈ [−0.30, 0.30] | 50.0 | 50.0 | Low |
| `zone_0` | Zone 0 (Green) | (−0.50, 0.55, 0°) | x ∈ [−0.85, −0.15], y ∈ [0.20, 0.90] | 50.0 | 20.0 | Low |
| `zone_1` | Zone 1 (Blue) | (2.05, 0.55, 180°) | x ∈ [1.70, 2.40], y ∈ [0.20, 0.90] | 95.0 | 30.0 | High |
| `zone_2` | Zone 2 (Yellow) | (2.05, −3.10, 180°) | x ∈ [1.70, 2.40], y ∈ [−3.45, −2.75] | 20.0 | 80.0 | Medium |
| `zone_3` | Zone 3 (Red) | (−0.50, −3.10, 0°) | x ∈ [−0.85, −0.15], y ∈ [−3.45, −2.75] | 50.0 | 45.0 | High |

The four zones are deliberately designed so they exhibit *different*
failure modes under weather pressure:

* `zone_0` (Green): nutrient-deficient, low runoff risk. Triggers the
  fertiliser path under sunny weather.
* `zone_1` (Blue): nearly saturated, high runoff risk. Frequently trips the
  SDG-14 halt when the weather turns wet.
* `zone_2` (Yellow): moisture-deficient, medium runoff risk. Triggers the
  irrigation path and occasionally the halt.
* `zone_3` (Red): near-healthy but high runoff risk. Becomes the second most
  common SDG-14 target.

Base station box is sized to never overlap a field zone's box (the
`ZoneDetector` would resolve to whichever is alphabetically first on
overlap, so this is defensive).

---

## 3. The colour map (Gazebo tiles + RViz)

`ZoneVisualizerNode` draws a tile in Gazebo anchored at the zone's
**target** coordinates (the same point the robot drives to). The colour
follows live telemetry:

| Colour | Category | Condition |
|---|---|---|
| Green | healthy | moisture ≥ 40, nutrients ≥ 50, vulnerability ≤ 70 |
| Blue | water | moisture < 40, nutrients ≥ 50, vulnerability ≤ 70 |
| Yellow | nutrient | moisture ≥ 40, nutrients < 50, vulnerability ≤ 70 |
| Orange | both | moisture < 40, nutrients < 50, vulnerability ≤ 70 |
| Red | risk | vulnerability > 70 (SDG-14 halt) |
| Grey | unknown | no telemetry yet |

In RViz (`ZoneDetectorNode`), the **bounding-box cube** is drawn at
`(min+max)/2` with extents `(max-min)`, alpha 0.4 by default and 0.8 for
the zone the robot is currently in.

---

## 4. Zone detector resolution

`ZoneDetectorNode.get_zone(x, y)` iterates the zones in **sorted key
order** and returns the first whose box contains the point. This is the
tie-break rule when boxes overlap: the alphabetically smallest wins. In
practice:

* `base_station` is alphabetically first.
* The four field zones are `zone_0`..`zone_3`, so they sort in numeric
  order.

The base station's box is intentionally disjoint from the field zones so
that in normal operation there is no overlap to resolve.

---

## 5. The base station

* The base station is the parking point for refills.
* In `CropDecisionNode._dispatch_to_base`, the target is offset by
  `±0.4 m` on Y per robot (so a future `robot_id=B` parks to the side
  and doesn't collide with the default `A` parking).
* On arrival (Nav2 `SUCCEEDED_AT_POSE`) the brain publishes
  `refill_resources` and the `RobotResourceNode` tops both meters to 100.
* A battery-fault (digital twin) keeps the battery clamped at 10 % and
  refills fertilizer only — this is what the demo uses to trigger a
  "RETURNING_TO_BASE" via the brain's resource guard.

---

## 6. Why ground-truth localization

`slam_toolbox` is still launched (Nav2 brings it up as a dependency), but
its map drifts badly in this small, symmetric room. The
`GroundTruthLocalizationNode` reads the robot's true pose from
`gz topic -e -t /world/default/dynamic_pose/info` and publishes a
`map→odom` transform that makes the `map` frame coincide with the world.
The robot is therefore always perfectly localised, and the Gazebo tiles
sit at exactly the right spots.

Nav2's global costmap is built from `/scan` (no static map layer), so
this swap costs nothing for planning.