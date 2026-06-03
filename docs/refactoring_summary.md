# Nutrient Nexus: Refactoring & Stabilization Summary

This document outlines the recent architectural improvements, bug fixes, and feature integrations applied to the Nutrient Nexus codebase. These changes were focused on eliminating technical debt, stabilizing multi-robot coordination, and completing the SDG-14 sustainability logic.

## 1. System Robustness & Parameterization
* **Explicit Robot Identification:** Removed fragile, hardcoded `get_namespace()` checks in `CropDecisionNode` and `RobotResourceNode`.
  * *Why:* Nodes now accept an explicitly declared `robot_id` parameter (e.g., `'A'` or `'B'`) from the launch files (`nexus.launch.py`, `nexus_dual.launch.py`). This guarantees deterministic behavior regardless of how ROS 2 namespaces are configured at runtime.
* **QoS Normalization:** Extracted duplicate `STATE_QOS` definitions from across the codebase into a centralized `my_turtlebot3_controller.qos` module.
  * *Why:* Ensures uniform Data Distribution Service (DDS) communication profiles across all nodes, preventing silent topic drops or mismatched reliability policies between publishers and subscribers.

## 2. Multi-Robot JSON Payload Handling
* **Robust Telemetry Parsing:** Updated `FieldSensorMockNode` and `SustainabilityAuditNode` to correctly parse JSON payloads on treatment topics (`/fertilise_zone`, `/irrigate_zone`).
  * *Why:* In the dual-robot setup, actions were previously just broadcasting strings (e.g., `"zone_0"`). By switching to JSON (`{"robot": "A", "zone": "zone_0"}`), the audit system can now accurately track and log *which* robot performed an action, fully supporting Robot B.

## 3. Environmental Sustainability (SDG-14 Integration)
* **Vulnerability Scoring:** Added dynamic `vulnerability` calculations to the `FieldSensorMockNode`. This score is calculated using the zone's base runoff risk, the current weather (e.g., storms increase risk), and current soil moisture.
* **Intervention Logic:** The `CropDecisionNode` now subscribes to `/field_vulnerability`. If a zone's vulnerability exceeds 70%, the node halts fertilization and publishes an `sdg14_intervention` message.
  * *Why:* This completes the core agricultural logic of the project, demonstrating proactive environmental protection by preventing nutrient runoff during heavy rain.

## 4. Navigation & Safety Tuning
* **LiDAR Logic Modularization:** Extracted duplicated LiDAR array slicing logic from `SafetyStopNode` and `TwinSafetyNode` into a new, shared `my_turtlebot3_controller.lidar_utils` module.
  * *Why:* Follows the DRY (Don't Repeat Yourself) principle. It reduces the code footprint by 40 lines and makes the safety logic easily testable.
* **Collision Prevention:** Increased the `SafetyStopNode`'s `stop_distance` from `0.14m` to `0.25m` across all launch files, and bumped the Nav2 `inflation_radius` to `0.40m`.
  * *Why:* The previous tolerances were too tight, causing the robots to scrape against the crop boundaries. The increased margins provide a much safer operating envelope for both simulated and physical robots.

## 5. Code Maintenance & Cleanup
* **Dead Code Removal:** Deleted legacy tutorial files and launch stubs (e.g., `MoveTurtleBot.py`, `DecisionNode.py`).
* **Documentation:** Updated `docs/doc.md` to remove stale, hardcoded paths (`/home/anas/...`) and aligned them with the active project workspace.
  * *Why:* Eliminates technical debt and prevents confusion for future developers or evaluators reviewing the repository.
