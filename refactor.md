# Refactor Plan — NutrientNexus

Full audit of `src/` based on reading every file. Items are grouped by category, ordered by impact.

---

## 1. Dead / Unnecessary Files

### Remove

| File | Reason |
|---|---|
| `src/__init__.py` | Empty, serves no purpose at the `src/` root level — ROS 2 packages are not Python packages at this level |
| `src/my_turtlebot3_controller/__init__.py` | Same — empty, redundant at the outer package wrapper level |
| `launch/without_map_launch.launch.py` | References `my_custom_turtlebot3_gazebo` which does not exist anywhere in this repo. Dead launch file |
| `launch/test_without.launch.py` | Also references the old `turtlebot3_gazebo` world launch directly, bypasses all custom infrastructure — appears to be a leftover from early development, superseded by `base.launch.py` |
| `navigation/MoveTurtleBot.py` | A 27-line "move forward forever" stub from the very first tutorial step. Has no role in the real system, is not called by any launch file, and its entry point `move_turtlebot` is only in `setup.py` as dead weight |
| `navigation/NavigationNode.py` | Another early tutorial stub — hard-codes a single goal to `(2.0, 0.0)`, sends it once, and exits. Completely superseded by `NavigationExecutorNode`. Entry point `navigation_node` in `setup.py` is dead |
| `algorithm/DecisionNode.py` | Original bin-collection decision node from an earlier iteration of the project. The system has fully moved to `CropDecisionNode` + `TwinSupervisorNode`. Entry point `DecisionNode` in `setup.py` is dead |
| `sensor/BinSensorMockNode.py` | Paired with `DecisionNode` — simulates filling bins. Since `DecisionNode` is dead, so is this. Entry point `BinSensorMockNode` is dead |
| `maps/map.pgm` / `maps/map.yaml` (in `my_tb3_world/maps/`) | These are inside the world package, but `nexus_real.launch.py` uses `my_turtlebot3_controller/maps/big_map.yaml`. The world package map files appear unused |

### Commented-out code

| File | Line(s) | Action |
|---|---|---|
| `config/zones.yaml` | Lines 19–31 | Old `zone_0` definition is entirely commented out. Remove it |

---

## 2. File Renames

Current naming is inconsistent: some files are named after their class (PascalCase), others after their role (snake_case). ROS 2 Python convention is `snake_case` for module files.

| Current path | Rename to | Reason |
|---|---|---|
| `my_turtlebot3_controller/CmdVelRelayNode.py` | `cmd_vel_relay_node.py` | PascalCase module file, inconsistent with rest |
| `my_turtlebot3_controller/RobotResourceNode.py` | `robot_resource_node.py` | Same |
| `navigation/MoveTurtleBot.py` | *(delete)* | See section 1 |
| `navigation/NavigationNode.py` | *(delete)* | See section 1 |
| `navigation/NavigationExecutorNode.py` | `navigation_executor_node.py` | PascalCase module |
| `navigation/SafetyStopNode.py` | `safety_stop_node.py` | PascalCase module |
| `navigation/ZoneDetectorNode.py` | `zone_detector_node.py` | PascalCase module |
| `navigation/odometry/OdomToGazeboPoseNode.py` | `odom_to_gazebo_pose_node.py` | PascalCase module |
| `navigation/odometry/OdomToTFNode.py` | `odom_to_tf_node.py` | PascalCase module |
| `algorithm/CropDecisionNode.py` | `crop_decision_node.py` | PascalCase module |
| `algorithm/DecisionNode.py` | *(delete)* | See section 1 |
| `sensor/BinSensorMockNode.py` | *(delete)* | See section 1 |
| `sensor/FieldSensorMockNode.py` | `field_sensor_mock_node.py` | PascalCase module |
| `dashboard/DashboardNode.py` | `dashboard_node.py` | PascalCase module |
| `twin/TwinSafetyNode.py` | `twin_safety_node.py` | PascalCase module |
| `twin/TwinSupervisorNode.py` | `twin_supervisor_node.py` | PascalCase module |
| `audit/SustainabilityAuditNode.py` | `sustainability_audit_node.py` | PascalCase module |
| `launch/without_map_launch.launch.py` | *(delete)* | See section 1 |
| `launch/test_without.launch.py` | *(delete)* | See section 1 |
| `worlds/new_world.world` | `agricultural_field.world` | `new_world` is a meaningless placeholder name |

After renaming modules, update `setup.py` entry points accordingly.

---

## 3. Package / Folder Renames

| Current name | Rename to | Reason |
|---|---|---|
| `my_turtlebot3_controller` (ROS package + all references) | `nutrient_nexus` | The project is called NutrientNexus. `my_turtlebot3_controller` is the default scaffolding name and reveals nothing about purpose. Affects `package.xml`, `setup.py`, `setup.cfg`, all `import` statements, all launch files, and the inner Python package directory |
| `my_tb3_world` (ROS package + all references) | `nutrient_nexus_world` | Same reasoning — `my_tb3_world` is a throwaway scaffolding name |
| `navigation/odometry/` | Keep, but consider merging into `navigation/` | Only two small files; a sub-sub-package adds nesting without benefit unless more odometry utilities are planned |

> **Note:** Renaming the ROS packages is a meaningful refactor with broad impact (CMakeLists, package.xml, all launch files, all imports, setup.py, resource marker file). Do it in a single focused commit.

---

## 4. Class / Function Renames

| Location | Current name | Rename to | Reason |
|---|---|---|---|
| `CmdVelRelayNode.py` | `CmdVelRelayNode` | Keep — but the log message `CmdVelRelayNode_PANA_DSP_PROJECT_FR1_Initialise` should become `CmdVelRelayNode started` | Log message contains leftover project metadata |
| `OdomToGazeboPoseNode.py` | `OdomToGazeboPoseNode` | `TwinPoseControllerNode` | The class name is misleading — it is a proportional feedback controller that makes the sim robot follow the real robot's pose. "OdomToGazeboPose" sounds like a converter, not a controller |
| `OdomToGazeboPoseNode.py` | `euler_from_quaternion` (module-level function) | Move into class or to a shared `utils.py` | Free-floating utility function; if `OdomToTFNode` or others ever need it too it will be duplicated |
| `NavigationExecutorNode.py` | `ten_second_wait_is_over_callback` (in `DecisionNode`) | `_bin_wait_complete` | The name is informal and encodes a magic number |
| `DecisionNode.py` | `make_decision_cycle` | *(delete with file)* | — |
| `ZoneDetectorNode.py` | `_package_share_or_source_dir` (module-level) | Move inline or to a shared `utils.py` | Free-floating helper duplicates the pattern used in `FieldSensorMockNode` |

---

## 5. Code Smell

### Duplicated `STATE_QOS` definition

`STATE_QOS` is copy-pasted into **six separate files** with slightly different `depth` values (1 vs 10), which is a silent inconsistency and a maintenance hazard:

- `CropDecisionNode.py` — depth 10
- `NavigationExecutorNode.py` — depth 1
- `ZoneDetectorNode.py` — depth 1
- `FieldSensorMockNode.py` — depth 10
- `DashboardNode.py` — depth 10
- `TwinSupervisorNode.py` — depth 10

**Fix:** Extract into a shared `qos.py` module (e.g. `nutrient_nexus/qos.py`) with named constants (`STATE_QOS_DEEP`, `STATE_QOS_SINGLE`) and import from there everywhere.

### Duplicated `_get_front_arc_distances` logic

The exact same LiDAR front-arc extraction loop exists in both `SafetyStopNode.py` and `TwinSafetyNode.py`. The only difference is the method name.

**Fix:** Extract to a shared `lidar_utils.py` (or `utils.py`) in the package root and import it in both nodes.

### Dead `pass` in empty callbacks

`CropDecisionNode.py` lines 88–91:
```python
def moisture_callback(self, msg):
    pass  # Using raw thresholds for simplicity right now

def nutrients_callback(self, msg):
    pass
```
These subscribe to topics but do nothing. Either implement them or remove the subscriptions entirely — subscribing and discarding is noise and consumes ROS resources.

### `_publish_status` called twice in sequence

`NavigationExecutorNode.py` lines 106–107:
```python
self._publish_status("REJECTED")
self._publish_status("IDLE")
```
And again lines 132–133:
```python
self._publish_status(final_status_str)
self._publish_status("IDLE")
```
Publishing two status values in the same tick means subscribers likely only see `IDLE`. The intermediate status (`REJECTED`, `ABORTED_NAVIGATION`, etc.) is published and immediately overwritten. This is a logic bug dressed up as code smell. **Fix:** Publish the final status, then schedule the `IDLE` transition via a short one-shot timer.

### Empty `finally` block

`NavigationExecutorNode.py` lines 149–150:
```python
finally:
    if hasattr(executor_node, '_action_client') and executor_node._action_client:
        pass
```
The `if` block does nothing. Remove it entirely.

### Hardcoded robot ID detection

Multiple nodes (e.g. `RobotResourceNode`, `CropDecisionNode`) use:
```python
self.robot_id = 'B' if self.get_namespace() == '/tb2' else 'A'
```
This is fragile — any namespace change silently makes both robots claim ID `A`. **Fix:** Declare a `robot_id` ROS parameter with a default, so it can be set explicitly in the launch file.

### Inconsistent `main()` signatures

`OdomToTFNode.py` line 29: `def main():` — missing `args=None` parameter that every other node uses. Harmless but inconsistent; ROS 2 entry points pass args.

### `SustainabilityAuditNode` subscribes to un-namespaced topics

`SustainabilityAuditNode` subscribes to `/fertilise_zone` and `/irrigate_zone` with absolute topic paths. In the dual-robot launch, Robot B publishes these under `/tb2/fertilise_zone`. The audit node will miss all Robot B events silently. **Fix:** Either subscribe to both namespaces explicitly, or restructure to receive events via a single aggregation topic.

### `TwinSupervisorNode._nav_status_cb` parses status as JSON

`_nav_status_cb` calls `json.loads(msg.data)` on the navigation status string, but `NavigationExecutorNode` publishes plain strings like `"NAVIGATING"` or `"IDLE"`, not JSON. The `json.loads` will always throw `JSONDecodeError` and silently fall through, meaning the supervisor never actually tracks navigation state. **Fix:** Read `msg.data` directly as a string.

### `setup.py` — TODO placeholders

`setup.py` has `description='TODO: Package description'` and `license='TODO: License declaration'`, as does `package.xml`. These should be filled in before the project is considered complete.

### `setup.py` entry point name inconsistency

Entry point `'DecisionNode = ...'` uses PascalCase while all others use `snake_case`. The dead entry points (`move_turtlebot`, `cmd_vel_relay_node`, `DecisionNode`, `BinSensorMockNode`, `navigation_node`) should be removed when their files are deleted.

---

## 6. Launch File Cleanup

- `nexus_dual.launch.py` has a duplicate section comment `# ── 8. Shared nodes` — the second `# ── 8.` should be `# ── 9.` (Twin Supervisor). Minor but confusing.
- `nexus_dual.launch.py` passes `robot_a_odom_topic` and `robot_b_odom_topic` as parameters to `TwinSupervisorNode`, but that node never declares or reads those parameters. Dead parameter noise.
- `base.launch.py` imports `sys` only for `sys.stderr` in two warning prints. These could simply use `print(..., file=sys.stderr)` or be replaced with proper Python `logging` / ROS logger calls.
- `nexus_real.launch.py` has no `TwinSupervisorNode` — the real-robot scenario loses the supervisor. If intentional, add a comment; if not, add the node.

---

## 7. Summary of Priority Order

1. **Delete dead files** (section 1) — zero risk, immediate noise reduction
2. **Fix `_publish_status` double-publish bug** (section 5) — actual behavioral bug
3. **Fix `TwinSupervisorNode._nav_status_cb` JSON parse bug** (section 5) — silent failure
4. **Fix `SustainabilityAuditNode` missing Robot B topics** (section 5) — data loss
5. **Extract `STATE_QOS` and LiDAR utils to shared modules** (section 5) — maintenance
6. **Rename module files to snake_case** (section 2) — consistency
7. **Rename packages** (section 3) — largest blast radius, do last
