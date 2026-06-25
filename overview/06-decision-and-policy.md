# Decision & Policy

The brain (`CropDecisionNode`) is a finite-state machine. This document
walks through every transition, the rules that fire, and the
sustainability guarantees.

---

## 1. The phase machine

```
             ┌──────────────────────────────────────┐
             ▼                                      │
   IDLE ─► WAITING_FOR_ASSIGNMENT                   │
                  │                                 │
                  ▼ assignment                      │
             NAVIGATING ─► VERIFYING_ZONE           │
                  │              │                   │
                  │              ▼ in-zone           │
                  │           SCANNING               │
                  │              │                   │
                  │              ▼ scan timer        │
                  │           DECIDING               │
                  │              │                   │
                  │       action taken?              │
                  │      /            \              │
                  │    yes             no            │
                  │     │               │            │
                  │     ▼               ▼            │
                  │  ACTUATING        COOLDOWN       │
                  │     │               │            │
                  │     ▼ timer         │            │
                  │  COOLDOWN ──────────┘            │
                  │       │                          │
                  │       ▼ more sub-targets?        │
                  └───── IDLE                       │
                                                     │
   any phase, battery/fert low OR /system_alerts ABORT:
                  ─► RETURNING_TO_BASE ─► (refill) ─► IDLE
```

### Phase semantics

| Phase | Trigger to enter | Trigger to leave |
|---|---|---|
| `IDLE` | initial / cycle complete | 1 Hz tick → publish zone request, set `WAITING_FOR_ASSIGNMENT` |
| `WAITING_FOR_ASSIGNMENT` | zone request published | `assignment_callback` → dispatch; or 5 s timeout → back to `IDLE` |
| `NAVIGATING` | goal published | `SUCCEEDED_AT_POSE` → `VERIFYING_ZONE`; failure status → `COOLDOWN` |
| `VERIFYING_ZONE` | Nav2 success | next tick: `physical_current_zone == active_zone_id` → `SCANNING`; else `COOLDOWN` |
| `SCANNING` | started scan timer | timer fires `_on_scan_complete` → `DECIDING` |
| `DECIDING` | scan done, in-flight action | publishes + sets `ACTUATING` or `COOLDOWN` |
| `ACTUATING` | treatment will run | `_actuation_tick` after `actuation_duration` → `COOLDOWN` |
| `COOLDOWN` | 1 s timer armed | timer → if sub-targets left: `NAVIGATING`; else `IDLE` |
| `RETURNING_TO_BASE` | dispatched to base | `SUCCEEDED_AT_POSE` → publish `refill_resources` → `IDLE` |

The resource-guard check runs at the top of every `state_machine_tick`
(1 Hz) and pre-empts *any* non-base phase to `RETURNING_TO_BASE` if
battery or fertilizer is critical.

### Timer invariants

* Exactly one of `_scan_timer`, `_actuation_timer`, `_cooldown_timer` is
  active at any time. Each is created in the phase that needs it and
  destroyed before the next phase's timer is created.
* The resource guard calls `_cancel_and_destroy` for all three before
  dispatching to base, then the base-state machine starts fresh.

---

## 2. The sustainability rules (SDG-14)

The "sustainability first" guarantee is enforced in two complementary
places: the `CropDecisionNode` decision step (does the right thing) and
the `SustainabilityAuditNode` (writes the proof).

### Rule A — Halt fertilisation on high runoff risk

In `_on_scan_complete`, **before** any fertiliser publish:

```python
if vuln > self.vulnerability_halt:           # default 70 %
    self.intervention_pub.publish(String(json.dumps({
        "robot":  self.robot_id,
        "zone":   self.active_zone_id,
        "action": "HALT_FERTILIZER",
        "reason": "High runoff risk",
    })))
    # ... but still check for irrigation below
elif nutri < self.nutrient_threshold:
    self.fertilise_pub.publish(...)
```

So a high-vulnerability zone will get **irrigated** if it is also dry, but
it will **never** be fertilised. The refusal is logged on
`/sdg14_intervention` and the audit node records it for the report.

### Rule B — Hard prohibition of actuation in the base station

```python
if self.active_zone_id == 'base_station' or self.physical_current_zone == 'base_station':
    self.get_logger().info('Zone is base station. Treatment is strictly forbidden here.')
    self._start_cooldown()
    return
```

A Nav2 false-positive that leaves the robot parked inside the base box
will not result in any treatment publish.

### Rule C — Predictive battery check

In `assignment_callback`, before accepting a zone:

```python
z  = self.raw_zones[zone_id]
if not self._can_afford_trip(z, base):
    self._dispatch_to_base()
    return
```

The trip estimate is:
```
dist_to_zone     = euclid(current → zone.target)
dist_zone_to_base= euclid(zone.target → base.target)
estimated_drain  = (dist_to_zone + dist_zone_to_base)
                  × battery_drain_per_meter
                  × battery_safety_margin      # default 1.5× (Nav2 path overhead)
```

If the estimate exceeds `battery_level` the brain refuses the assignment
and heads home. This prevents a mid-zone stranding.

### Rule D — Storm abort

`TwinSupervisorNode._weather_cb` watches `/weather_forecast`; on
`storm` it broadcasts a global `/system_alerts = ABORT/STORM_EMERGENCY`.
`CropDecisionNode.alerts_callback` clears the sub-target queue and
dispatches to base. Even after recovery, the supervisor continues to
send the robot back to base until the weather changes.

---

## 3. The vulnerability model

`FieldSensorMockNode` recomputes vulnerability every tick:

```
vulnerability = min(100,
                    base_risk      # 0.2 / 0.5 / 0.8 for Low / Medium / High
                  × weather_factor # 1.5 / 2.0 / 0.8 / 0.3 for rainy/storm/overcast/sunny
                  × moisture       # 0..1
                  × 100)
```

So a "High" zone in storm with wet soil saturates the 0..100 scale; the
SDG-14 halt threshold (70) is reached quickly under realistic
conditions. The four zones are seeded to demonstrate this:

| Zone | Baseline | Under sunny | Under storm (wet soil) |
|---|---|---|---|
| `zone_1` (Blue, High) | 95 % moisture | vulnerability ≈ 22 | saturation 100 → halt |
| `zone_3` (Red, High) | 50 % moisture | vulnerability ≈ 12 | ≈ 80 → halt |
| `zone_2` (Yellow, Medium) | 20 % moisture | vulnerability ≈ 3 | ≈ 40 → safe |
| `zone_0` (Green, Low) | 50 % moisture | vulnerability ≈ 3 | ≈ 40 → safe |

This makes the demo predictable: a single click of *Storm* on the
dashboard reliably produces SDG-14 halts on zones 1 and 3.

---

## 4. The actuator (spin-in-place)

When `_on_scan_complete` decides an action is needed, it enters
`ACTUATING` and starts a 0.1 s timer. For each tick the timer runs:

```python
cmd = Twist()
cmd.angular.z = 0.6
self.treatment_vel_pub.publish(cmd)   # cmd_vel_nav
```

This is a **visible** behaviour — the robot spins in place for
`actuation_duration_sec` (default 2 s) to mime the spray. The
`/cmd_vel_nav` channel is muxed with Nav2's `cmd_vel_nav` into
`/cmd_vel_raw`, then `SafetyStopNode` gates it before reaching the
robot. So a wall in front during the spray is still respected.

The control duration is constant in the current build; the documented
spec mentions "dynamic duration / speed based on deficit and risk" but
the implementation is uniform per treatment. The structure makes the
extension trivial — `_actuation_start_time` is a member variable and the
duration is already parameterised.

---

## 5. The actuator (irrigation)

Irrigation is not a "movement" — it's a publish-only event. The brain
publishes `{"robot":…, "zone":…}` on `/irrigate_zone`; the
`FieldSensorMockNode` raises the zone's moisture to
`irrigate_replenish_pct` (default 95) and immediately republishes
telemetry. The robot does not spin during irrigation (only during
fertilisation in the current build).

---

## 6. Why a state machine at all?

A robot with a single supervisor call would either spin forever waiting
for a reply or flood the supervisor with retries on every anomaly. The
FSM makes the wait explicit, makes failures recoverable (each phase has
its own recovery), and decouples the brain from the supervisor's
internal state. The timer discipline (exactly one of
scan/actuation/cooldown active) makes the system safe to interrupt —
even a hard `Ctrl-C` mid-actuation leaves the timers correctly torn
down by the `_cancel_and_destroy` discipline when the next phase
enters.