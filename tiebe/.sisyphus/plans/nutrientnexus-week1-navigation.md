# NutrientNexus Week 1: ROS2 Navigation Foundation

## TL;DR

> **Quick Summary**: Build ROS2 Python package enabling TurtleBot3 Burger autonomous navigation in 4m x 4m environment with crop zones, working in both Gazebo simulation and real robot.
> 
> **Deliverables**:
> - ROS2 package `nutrient_nexus_navigation` with ament_python build
> - Gazebo world (4x4m box, walls, center obstacle)
> - SLAM + Nav2 autonomous navigation
> - Zone detection system (3 coordinate-based zones)
> - Launch files for sim and real robot
> - RViz visualization with zone markers
> - Unit + integration tests
> 
> **Estimated Effort**: Medium (2-3 days for ROS2-experienced developer)
> **Parallel Execution**: YES - 3 waves
> **Critical Path**: Task 1 → Task 5 → Task 8 → Task 11

---

## Context

### Original Request
User needs Week 1 foundation for NutrientNexus PoC: ROS2 navigation system for TurtleBot3 Burger that works in both simulation and real robot. This is part of a larger Digital Twin project for agricultural nutrient management, but Week 1 focuses solely on navigation infrastructure.

### Interview Summary
**Key Discussions**:
- Environment: 4m x 4m wooden box with walls + single center obstacle
- Zones: 2-3 coordinate-based crop zones (axis-aligned bounding boxes)
- Week 1 scope: Navigation only, sensor/decision logic deferred
- Must work on real robot (sim-to-real ready from day 1)
- Real robot validation: Manual testing acceptable (physical hardware, human operator)
- Digital twin communication framework defined (topics/interfaces), but data flow deferred

**Research Findings**:
- Codebase is clean slate (no existing ROS2 code)
- ROS2 Jazzy + TurtleBot3 patterns identified from flake.nix
- Nav2 + SLAM toolbox integration patterns documented
- Sim-to-real strategy: use_sim_time + hardware swap

### Metis Review
**Identified Gaps** (addressed):
- Zone detection precision: Odometry drift acceptable (10-20cm), no events needed
- Obstacle handling: Nav2 costmap handles dynamically
- Real robot acceptance: Code runs without errors, basic navigation works (perfect tuning deferred)
- Gazebo fidelity: Approximate dimensions sufficient
- Launch files: Separate sim/real files (no premature abstraction)
- Zone format: Axis-aligned bounding boxes (NOT polygons)
- Package name: `nutrient_nexus_navigation` (NOT `nutrientnexus_poc`)

---

## Work Objectives

### Core Objective
Build a ROS2 Python navigation package that enables TurtleBot3 Burger to autonomously map and navigate a 4m x 4m environment with crop zones, working in both Gazebo simulation and on the real robot.

### Concrete Deliverables
- ROS2 package: `nutrient_nexus_navigation/` with setup.py, package.xml
- Gazebo world file: `worlds/farm_box.world`
- Zone detection node: `zone_detector.py` publishing to `/current_zone`
- Launch files: `sim_bringup.launch.py`, `real_bringup.launch.py`
- Config files: `config/zones.yaml`, `config/nav2_params.yaml`
- RViz config: `rviz/nav.rviz`
- Tests: `test/test_zone_detection.py`, `test/test_sim_navigation.py`
- Documentation: `README.md` with setup, launch, testing instructions

### Definition of Done
- [ ] `ros2 launch nutrient_nexus_navigation sim_bringup.launch.py` starts without errors
- [ ] Robot navigates autonomously to all 3 zones in Gazebo
- [ ] `/current_zone` topic publishes correct zone names
- [ ] `pytest test/` passes all tests
- [ ] `ros2 launch nutrient_nexus_navigation real_bringup.launch.py` starts on real robot
- [ ] Real robot responds to nav goals (perfect navigation not required)

### Must Have
- Single monolithic Python package (no package splitting)
- SLAM toolbox + Nav2 stack (no custom path planning)
- Axis-aligned bounding box zones in YAML
- Hardcoded `TURTLEBOT3_MODEL=burger` in launch files
- `use_sim_time` correctly set (true for sim, false for real)
- Zone detection as simple coordinate check (no polygon math)
- Agent-executable QA scenarios for automated tasks (Tasks 1-11)
- Evidence files saved to `.sisyphus/evidence/`
- Real robot validation (Task 12) and final QA (F3) are manual exceptions (physical hardware required)

### Must NOT Have (Guardrails)
- **No sensor data processing** (stub topics only, no logic)
- **No decision engine logic** (framework only)
- **No custom ROS2 message types** (use std_msgs for Week 1)
- **No map save/load** (SLAM rebuilds map each run)
- **No perfect Nav2 tuning** for real robot (accept "good enough")
- **No separate packages** for sim/real/common
- **No abstract factories, strategy patterns** (avoid over-engineering)
- **No custom exception hierarchies** (Python built-ins sufficient)
- **No retry logic** beyond Nav2 defaults
- **No web dashboard or GUI** (RViz only)
- **No polygon-based zones** (bounding boxes only)

---

## Verification Strategy (MANDATORY)

> **AUTOMATED VERIFICATION** for Tasks 1-11: ALL verification is agent-executed.
> **MANUAL EXCEPTIONS** for Task 12 (real robot) and F3 (final QA): Physical hardware requires human interaction.
> Acceptance criteria for Tasks 1-11 must be fully automated. Task 12 and F3 explicitly allow manual steps.

### Test Decision
- **Infrastructure exists**: NO (will be created)
- **Automated tests**: Tests-after (implementation first, then tests)
- **Framework**: pytest (unit tests) + launch_testing (integration tests)
- **Agent QA**: MANDATORY for all tasks (see QA Policy below)

### QA Policy
Tasks 1-11 MUST include agent-executed QA scenarios. Task 12 (real robot validation) uses manual QA scenarios due to physical hardware requirements.
Evidence saved to `.sisyphus/evidence/task-{N}-{scenario-slug}.{ext}`.

- **ROS nodes**: Use Bash (ros2 commands) - `ros2 topic echo`, `ros2 node list`, `ros2 launch`
- **CLI/TUI**: Use interactive_bash (tmux) - Run commands, validate output
- **Python modules**: Use Bash (pytest) - Import, call functions, compare output
- **Integration**: Use Bash (launch + topic checks) - Start system, verify topics, send goals

---

## Execution Strategy

### Parallel Execution Waves

> Maximize throughput by grouping independent tasks into parallel waves.
> Each wave completes before the next begins.
> Target: 5-8 tasks per wave. Fewer than 3 per wave (except final) = under-splitting.

```
Wave 1 (Start Immediately - foundation):
├── Task 1: Package scaffolding [quick]
├── Task 2: Gazebo world file [quick]
└── Task 3: Zone config YAML [quick]

Wave 2 (After Wave 1 - core implementation):
├── Task 4: Zone detector node [quick]
├── Task 5: sim_bringup launch file [unspecified-high]
├── Task 6: real_bringup launch file [unspecified-high]
├── Task 7: Nav2 params config [quick]
└── Task 8: RViz config [quick]

Wave 3 (After Wave 2 - testing & docs):
├── Task 9: Unit tests for zone detection [quick]
├── Task 10: Integration test for sim navigation [unspecified-high]
├── Task 11: README documentation [writing]
└── Task 12: Real robot validation [unspecified-high]

Wave FINAL (After ALL tasks - 4 parallel reviews):
├── Task F1: Plan compliance audit (oracle)
├── Task F2: Code quality review (unspecified-high)
├── Task F3: Real manual QA (unspecified-high)
└── Task F4: Scope fidelity check (deep)
-> Present results -> Get explicit user okay
```

**Critical Path**: Task 1 → Task 5 → Task 8 → Task 11 → F1-F4 → user okay
**Parallel Speedup**: ~60% faster than sequential
**Max Concurrent**: 4 (Wave 2)

### Dependency Matrix

| Task | Depends On | Blocks | Wave |
|------|-----------|--------|------|
| 1 | - | 2,3,4,5,6,7,8 | 1 |
| 2 | 1 | 5 | 1 |
| 3 | 1 | 4 | 1 |
| 4 | 1,3 | 5,6,9 | 2 |
| 5 | 1,2,4 | 10 | 2 |
| 6 | 1,4 | 12 | 2 |
| 7 | 1 | 5,6 | 2 |
| 8 | 1 | 10,11 | 2 |
| 9 | 4 | - | 3 |
| 10 | 5,8 | - | 3 |
| 11 | 8 | - | 3 |
| 12 | 6 | - | 3 |
| F1-F4 | 1-12 | - | FINAL |

### Agent Dispatch Summary

- **Wave 1**: 3 tasks - T1 → `quick`, T2 → `quick`, T3 → `quick`
- **Wave 2**: 5 tasks - T4 → `quick`, T5 → `unspecified-high`, T6 → `unspecified-high`, T7 → `quick`, T8 → `quick`
- **Wave 3**: 4 tasks - T9 → `quick`, T10 → `unspecified-high`, T11 → `writing`, T12 → `unspecified-high`
- **Wave FINAL**: 4 tasks - F1 → `oracle`, F2 → `unspecified-high`, F3 → `unspecified-high`, F4 → `deep`

---

## TODOs

> Implementation + Test = ONE Task. Never separate.
> EVERY task MUST have: Recommended Agent Profile + Parallelization info + QA Scenarios.
> **A task WITHOUT QA Scenarios is INCOMPLETE. No exceptions.**

- [x] 1. Package Scaffolding

  **What to do**:
  - Create ROS2 Python package structure using ament_python
  - Package name: `nutrient_nexus_navigation`
  - Directory structure:
    ```
    nutrient_nexus_navigation/
    ├── nutrient_nexus_navigation/
    │   ├── __init__.py
    │   └── (nodes will go here)
    ├── launch/
    ├── config/
    ├── worlds/
    ├── rviz/
    ├── test/
    ├── setup.py
    ├── package.xml
    └── README.md
    ```
  - setup.py: Configure entry points, install launch/config/worlds/rviz directories
  - package.xml: Dependencies (rclpy, nav2_bringup, slam_toolbox, turtlebot3_gazebo, std_msgs, geometry_msgs)

  **Must NOT do**:
  - Create separate packages for sim/real/common
  - Add custom message definitions
  - Create abstract base classes or factories

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Standard ROS2 package scaffolding, well-documented pattern
  - **Skills**: []
    - No specialized skills needed for basic package structure

  **Parallelization**:
  - **Can Run In Parallel**: NO (foundation task)
  - **Parallel Group**: Wave 1 (with Tasks 2, 3 after this completes)
  - **Blocks**: Tasks 2, 3, 4, 5, 6, 7, 8
  - **Blocked By**: None (can start immediately)

  **References**:
  - Official docs: https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html - ament_python package structure
  - Official docs: https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html - Launch file installation in setup.py
  - Pattern: TurtleBot3 packages use `data_files` in setup.py to install launch/config/worlds

  **Acceptance Criteria**:
  - [ ] Directory structure created with all required folders
  - [ ] setup.py configures package name, version, dependencies, data_files
  - [ ] package.xml lists all ROS2 dependencies
  - [ ] `colcon build --packages-select nutrient_nexus_navigation` succeeds
  - [ ] `ros2 pkg prefix nutrient_nexus_navigation` returns package path

  **QA Scenarios (MANDATORY)**:

  ```
  Scenario: Package builds successfully
    Tool: Bash
    Preconditions: Clean workspace, no previous build
    Steps:
      1. cd /home/tiebe/CBL
      2. colcon build --packages-select nutrient_nexus_navigation
      3. Check exit code: echo $?
      4. Source workspace: source install/setup.bash
      5. Verify package: ros2 pkg prefix nutrient_nexus_navigation
    Expected Result: 
      - colcon build exit code 0
      - ros2 pkg prefix returns path ending in /install/nutrient_nexus_navigation
    Failure Indicators:
      - Non-zero exit code
      - "Package not found" error
    Evidence: .sisyphus/evidence/task-1-build-success.txt

  Scenario: Package dependencies declared
    Tool: Bash
    Preconditions: Package built
    Steps:
      1. cat nutrient_nexus_navigation/package.xml | grep -E "(rclpy|nav2_bringup|slam_toolbox|turtlebot3)"
      2. Count dependencies: cat nutrient_nexus_navigation/package.xml | grep "<depend>" | wc -l
    Expected Result:
      - All required dependencies present (rclpy, nav2_bringup, slam_toolbox, turtlebot3_gazebo)
      - At least 8 dependencies declared
    Failure Indicators:
      - Missing critical dependencies
      - Fewer than 8 dependencies
    Evidence: .sisyphus/evidence/task-1-dependencies.txt
  ```

  **Evidence to Capture**:
  - [ ] task-1-build-success.txt (colcon build output)
  - [ ] task-1-dependencies.txt (package.xml grep output)

  **Commit**: YES
  - Message: `feat(setup): add ROS2 package scaffolding`
  - Files: `setup.py, package.xml, directory structure`
  - Pre-commit: `colcon build --packages-select nutrient_nexus_navigation`

- [x] 2. Gazebo World File

  **What to do**:
  - Create `worlds/farm_box.world` Gazebo world file
  - Prefer a simple 4m x 4m box with walls and a single center obstacle unless an approved testing-environment world asset is provided later
  - If a provided testing world is approved for reuse, adapt it to the Week 1 TurtleBot3 workflow without adding non-navigation behavior
  - Keep all environment geometry static and suitable for SLAM + Nav2 bringup
  - Include ground plane, lighting, and physics configuration

  **Must NOT do**:
  - Add custom runtime behavior unrelated to navigation environment setup
  - Add moving obstacles or simulation-specific business logic
  - Drift into application-specific sensing/decision behavior

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Standard Gazebo world file, simple geometry
  - **Skills**: []
    - No specialized skills needed

  **Parallelization**:
  - **Can Run In Parallel**: YES (after Task 1)
  - **Parallel Group**: Wave 1 (with Task 3)
  - **Blocks**: Task 5 (sim launch file needs world)
  - **Blocked By**: Task 1 (needs package structure)

  **References**:
  - Pattern: `turtlebot3_gazebo/worlds/turtlebot3_world.world` - Basic world structure
  - Official docs: https://classic.gazebosim.org/tutorials?tut=build_world - World file syntax
  - Gazebo models: Use `<model>` tags with basic shapes (box, cylinder)

  **Acceptance Criteria**:
  - [ ] World file created at `worlds/farm_box.world`
  - [ ] World file parses as valid SDF/XML and is usable by simulation bringup
  - [ ] Ground plane present
  - [ ] Static obstacle geometry present for navigation testing
  - [ ] Final world matches the approved testing environment used by the project

  **QA Scenarios (MANDATORY)**:

  ```
  Scenario: World file parses and is launch-ready
    Tool: Bash
    Preconditions: Gazebo installed, world file exists
    Steps:
      1. python3 -c "import xml.etree.ElementTree as ET; ET.parse('worlds/farm_box.world'); print('XML valid')" 2>&1 | tee /tmp/gazebo_output.txt
      2. grep -i "XML valid" /tmp/gazebo_output.txt
      3. grep -i "error" /tmp/gazebo_output.txt
    Expected Result:
      - XML parses successfully
      - No parse or file errors
    Failure Indicators:
      - Parse error or missing file
    Evidence: .sisyphus/evidence/task-2-gazebo-load.txt

  Scenario: World geometry validation
    Tool: Bash
    Preconditions: World file exists
    Steps:
      1. grep -c "<model" worlds/farm_box.world
      2. grep -c "<static>true</static>" worlds/farm_box.world
      3. grep -c "ground_plane" worlds/farm_box.world
    Expected Result:
      - At least 5 models (ground + obstacles)
      - Static geometry appears multiple times
      - "ground_plane" appears once
    Failure Indicators:
      - Fewer than 5 models
      - Missing ground plane
    Evidence: .sisyphus/evidence/task-2-geometry.txt
  ```

  **Evidence to Capture**:
  - [ ] task-2-gazebo-load.txt (Gazebo startup output)
  - [ ] task-2-geometry.txt (model count validation)

  **Commit**: YES
  - Message: `feat(sim): add Gazebo farm box world`
  - Files: `worlds/farm_box.world`
  - Pre-commit: `gazebo worlds/farm_box.world --verbose` (manual check)

- [x] 3. Zone Configuration YAML

  **What to do**:
  - Create `config/zones.yaml` with 3 crop zones
  - Use axis-aligned bounding boxes (NOT polygons)
  - Format: `zone_a: {min_x: 0.5, max_x: 1.5, min_y: 0.5, max_y: 1.5}`
  - Zones should fit within 4m x 4m world, avoid center obstacle
  - Example layout:
    - zone_a: Bottom-left quadrant
    - zone_b: Top-right quadrant
    - zone_c: Top-left quadrant
  - Add comments explaining coordinate system (origin at world center)

  **Must NOT do**:
  - Use polygon coordinates (only bounding boxes)
  - Create overlapping zones
  - Add zone transition events or callbacks

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Simple YAML file with coordinate definitions
  - **Skills**: []
    - No specialized skills needed

  **Parallelization**:
  - **Can Run In Parallel**: YES (after Task 1)
  - **Parallel Group**: Wave 1 (with Task 2)
  - **Blocks**: Task 4 (zone detector needs config)
  - **Blocked By**: Task 1 (needs config/ directory)

  **References**:
  - Pattern: ROS2 YAML configs use nested dictionaries
  - Coordinate system: Gazebo uses X-forward, Y-left, Z-up (right-hand rule)

  **Acceptance Criteria**:
  - [ ] Config file created at `config/zones.yaml`
  - [ ] 3 zones defined (zone_a, zone_b, zone_c)
  - [ ] Each zone has min_x, max_x, min_y, max_y
  - [ ] No zones overlap
  - [ ] All zones within 4m x 4m bounds (-2 to 2 on X and Y)

  **QA Scenarios (MANDATORY)**:

  ```
  Scenario: YAML syntax validation
    Tool: Bash
    Preconditions: Config file exists
    Steps:
      1. python3 -c "import yaml; yaml.safe_load(open('config/zones.yaml'))"
      2. echo $?
    Expected Result:
      - Exit code 0 (valid YAML)
      - No syntax errors
    Failure Indicators:
      - Non-zero exit code
      - YAML parse error
    Evidence: .sisyphus/evidence/task-3-yaml-valid.txt

  Scenario: Zone bounds validation
    Tool: Bash
    Preconditions: Config file exists
    Steps:
      1. python3 -c "
import yaml
zones = yaml.safe_load(open('config/zones.yaml'))
for name, zone in zones.items():
    assert 'min_x' in zone and 'max_x' in zone
    assert 'min_y' in zone and 'max_y' in zone
    assert zone['min_x'] < zone['max_x']
    assert zone['min_y'] < zone['max_y']
    assert -2 <= zone['min_x'] <= 2
    assert -2 <= zone['max_x'] <= 2
    print(f'{name}: OK')
"
    Expected Result:
      - All zones print "OK"
      - No assertion errors
    Failure Indicators:
      - AssertionError
      - Missing keys
    Evidence: .sisyphus/evidence/task-3-bounds-valid.txt
  ```

  **Evidence to Capture**:
  - [ ] task-3-yaml-valid.txt (YAML parse output)
  - [ ] task-3-bounds-valid.txt (bounds validation output)

  **Commit**: YES
  - Message: `feat(config): add zone definitions`
  - Files: `config/zones.yaml`
  - Pre-commit: `python3 -c "import yaml; yaml.safe_load(open('config/zones.yaml'))"`

- [x] 4. Zone Detector Node

  **What to do**:
  - Create `nutrient_nexus_navigation/zone_detector.py` ROS2 node
  - Subscribe to `/odom` (nav_msgs/Odometry) for robot position
  - Load zones from `config/zones.yaml` using `ament_index_python`
  - Check if robot position is within any zone (simple coordinate comparison)
  - Publish current zone to `/current_zone` (std_msgs/String)
  - If outside all zones, publish "no_zone"
  - Update rate: 2 Hz (sufficient for navigation)

  **Must NOT do**:
  - Implement polygon math or complex geometry
  - Add zone transition events or callbacks
  - Create abstract zone detector interface
  - Add retry logic or error recovery

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Simple ROS2 node with coordinate checking
  - **Skills**: []
    - No specialized skills needed

  **Parallelization**:
  - **Can Run In Parallel**: YES (after Tasks 1, 3)
  - **Parallel Group**: Wave 2 (with Tasks 5, 6, 7, 8)
  - **Blocks**: Tasks 5, 6, 9 (launch files and tests need this node)
  - **Blocked By**: Tasks 1 (package structure), 3 (zone config)

  **References**:
  - Pattern: ROS2 Python nodes use `rclpy.node.Node` base class
  - API: `ament_index_python.packages.get_package_share_directory()` for config loading
  - Topic: `/odom` publishes `nav_msgs/Odometry` with `pose.pose.position.x/y`
  - Example: TurtleBot3 nodes for subscription patterns

  **Acceptance Criteria**:
  - [ ] Node file created at `nutrient_nexus_navigation/zone_detector.py`
  - [ ] Node loads zones from YAML without errors
  - [ ] Node subscribes to `/odom` topic
  - [ ] Node publishes to `/current_zone` topic
  - [ ] Entry point added to setup.py: `zone_detector = nutrient_nexus_navigation.zone_detector:main`

  **QA Scenarios (MANDATORY)**:

  ```
  Scenario: Node starts without errors
    Tool: Bash
    Preconditions: Package built, config exists
    Steps:
      1. source install/setup.bash
      2. timeout 10s ros2 run nutrient_nexus_navigation zone_detector 2>&1 | tee /tmp/zone_detector.txt &
      3. sleep 3
      4. ros2 node list | grep zone_detector
      5. pkill -9 -f zone_detector
      6. grep -i "error" /tmp/zone_detector.txt
    Expected Result:
      - Node appears in `ros2 node list`
      - No "error" in output
    Failure Indicators:
      - Node not in list
      - Error messages in output
    Evidence: .sisyphus/evidence/task-4-node-start.txt

  Scenario: Zone detection logic
    Tool: Bash (pytest unit test)
    Preconditions: Node code exists
    Steps:
      1. Create test file with mock positions
      2. Test position (1.0, 1.0) → should return zone_a
      3. Test position (0.0, 0.0) → should return "no_zone"
      4. pytest test/test_zone_detection.py::test_zone_logic -v
    Expected Result:
      - All position tests pass
      - Correct zone returned for each position
    Failure Indicators:
      - Test failures
      - Wrong zone returned
    Evidence: .sisyphus/evidence/task-4-zone-logic.txt
  ```

  **Evidence to Capture**:
  - [ ] task-4-node-start.txt (node startup output)
  - [ ] task-4-zone-logic.txt (unit test output)

  **Commit**: YES
  - Message: `feat(nav): add zone detector node`
  - Files: `nutrient_nexus_navigation/zone_detector.py, setup.py (entry point)`
  - Pre-commit: `colcon build --packages-select nutrient_nexus_navigation && source install/setup.bash && ros2 run nutrient_nexus_navigation zone_detector --help`

- [x] 5. Simulation Launch File

  **What to do**:
  - Create `launch/sim_bringup.launch.py`
  - Launch components:
    1. Gazebo with `farm_box.world`
    2. TurtleBot3 spawn (TURTLEBOT3_MODEL=burger hardcoded)
    3. robot_state_publisher (URDF from turtlebot3_description)
    4. SLAM Toolbox (slam_toolbox/online_async_launch.py)
    5. Nav2 (nav2_bringup/navigation_launch.py)
    6. Zone detector node
  - Set `use_sim_time:=True` for all nodes
  - Load `config/nav2_params.yaml` for Nav2
  - Use `IncludeLaunchDescription` for Nav2/SLAM (don't manually launch nodes)

  **Must NOT do**:
  - Create common_bringup.launch.py (no premature abstraction)
  - Manually launch Nav2 nodes (use nav2_bringup)
  - Add custom path planning or recovery behaviors

  **Recommended Agent Profile**:
  - **Category**: `unspecified-high`
    - Reason: Complex launch file with multiple subsystems, requires ROS2 launch API knowledge
  - **Skills**: []
    - No specialized skills needed, but higher complexity than quick tasks

  **Parallelization**:
  - **Can Run In Parallel**: YES (after Tasks 1, 2, 4)
  - **Parallel Group**: Wave 2 (with Tasks 6, 7, 8)
  - **Blocks**: Task 10 (integration test needs sim launch)
  - **Blocked By**: Tasks 1 (package), 2 (world), 4 (zone detector)

  **References**:
  - Pattern: `turtlebot3_gazebo/launch/turtlebot3_world.launch.py` - Gazebo + robot spawn
  - Pattern: `nav2_bringup/launch/bringup_launch.py` - Nav2 + SLAM integration
  - API: `launch.actions.IncludeLaunchDescription` for including other launch files
  - API: `launch.substitutions.PathJoinSubstitution` for file paths
  - Critical: Set `use_sim_time` parameter for all nodes

  **Acceptance Criteria**:
  - [ ] Launch file created at `launch/sim_bringup.launch.py`
  - [ ] `ros2 launch nutrient_nexus_navigation sim_bringup.launch.py` starts without errors
  - [ ] Gazebo opens with farm_box world and TurtleBot3
  - [ ] All required nodes running: `ros2 node list` shows slam_toolbox, nav2, zone_detector
  - [ ] `/map` topic publishes (SLAM working): `ros2 topic hz /map`
  - [ ] `/current_zone` topic publishes: `ros2 topic echo /current_zone --once`

  **QA Scenarios (MANDATORY)**:

  ```
  Scenario: Launch file starts all components
    Tool: Bash
    Preconditions: Package built, all dependencies installed
    Steps:
      1. source install/setup.bash
      2. timeout 60s ros2 launch nutrient_nexus_navigation sim_bringup.launch.py 2>&1 | tee /tmp/sim_launch.txt &
      3. sleep 30 (wait for Gazebo + SLAM initialization)
      4. ros2 node list > /tmp/node_list.txt
      5. grep -E "(slam_toolbox|nav2|zone_detector)" /tmp/node_list.txt
      6. ros2 topic list > /tmp/topic_list.txt
      7. grep -E "(/map|/current_zone|/cmd_vel|/scan)" /tmp/topic_list.txt
      8. pkill -9 -f "ros2 launch"
      9. pkill -9 gazebo
    Expected Result:
      - slam_toolbox node present
      - nav2 nodes present (at least 3)
      - zone_detector node present
      - /map, /current_zone, /cmd_vel, /scan topics exist
    Failure Indicators:
      - Missing nodes
      - Missing topics
      - Launch errors in output
    Evidence: .sisyphus/evidence/task-5-sim-launch.txt

  Scenario: SLAM publishes map
    Tool: Bash
    Preconditions: Sim launch running
    Steps:
      1. source install/setup.bash
      2. timeout 60s ros2 launch nutrient_nexus_navigation sim_bringup.launch.py &
      3. sleep 30
      4. ros2 topic hz /map --window 10 2>&1 | head -20
      5. pkill -9 -f "ros2 launch"
      6. pkill -9 gazebo
    Expected Result:
      - /map topic publishes at ~1 Hz or slower (SLAM updates)
      - No "WARNING: no messages received" error
    Failure Indicators:
      - No messages on /map
      - Topic doesn't exist
    Evidence: .sisyphus/evidence/task-5-slam-map.txt
  ```

  **Evidence to Capture**:
  - [ ] task-5-sim-launch.txt (launch output + node/topic lists)
  - [ ] task-5-slam-map.txt (map topic rate check)

  **Commit**: YES
  - Message: `feat(sim): add simulation launch file`
  - Files: `launch/sim_bringup.launch.py`
  - Pre-commit: `ros2 launch nutrient_nexus_navigation sim_bringup.launch.py` (manual check, kill after 30s)

- [x] 6. Real Robot Launch File

  **What to do**:
  - Create `launch/real_bringup.launch.py`
  - Launch components:
    1. TurtleBot3 robot drivers (turtlebot3_bringup/robot.launch.py)
    2. robot_state_publisher (URDF from turtlebot3_description)
    3. SLAM Toolbox (slam_toolbox/online_async_launch.py)
    4. Nav2 (nav2_bringup/navigation_launch.py)
    5. Zone detector node
  - Set `use_sim_time:=False` for all nodes
  - Load same `config/nav2_params.yaml` as sim
  - Hardcode `TURTLEBOT3_MODEL=burger`

  **Must NOT do**:
  - Share launch file with sim (keep separate for clarity)
  - Add custom recovery behaviors
  - Implement battery monitoring or charging logic

  **Recommended Agent Profile**:
  - **Category**: `unspecified-high`
    - Reason: Similar complexity to sim launch, but real hardware considerations
  - **Skills**: []
    - No specialized skills needed

  **Parallelization**:
  - **Can Run In Parallel**: YES (after Tasks 1, 4)
  - **Parallel Group**: Wave 2 (with Tasks 5, 7, 8)
  - **Blocks**: Task 12 (real robot validation needs this)
  - **Blocked By**: Tasks 1 (package), 4 (zone detector)

  **References**:
  - Pattern: `turtlebot3_bringup/launch/robot.launch.py` - Real robot drivers
  - Pattern: Same Nav2/SLAM structure as sim launch, but `use_sim_time:=False`
  - Critical: Ensure ROS_DOMAIN_ID matches between laptop and robot

  **Acceptance Criteria**:
  - [ ] Launch file created at `launch/real_bringup.launch.py`
  - [ ] `use_sim_time` parameter set to False (verified by grep)
  - [ ] Launch file includes turtlebot3_bringup, nav2_bringup, slam_toolbox (verified by grep)
  - [ ] nav2_params.yaml path matches sim launch file (verified by grep)
  - [ ] Python syntax valid: `python3 -m py_compile launch/real_bringup.launch.py` exits 0

  **QA Scenarios (MANDATORY)**:

  ```
  Scenario: Launch file syntax validation
    Tool: Bash
    Preconditions: Package built
    Steps:
      1. python3 -m py_compile launch/real_bringup.launch.py
      2. echo $?
      3. grep "use_sim_time.*False" launch/real_bringup.launch.py
    Expected Result:
      - Compilation succeeds (exit code 0)
      - use_sim_time set to False
    Failure Indicators:
      - Syntax errors
      - use_sim_time not set or set to True
    Evidence: .sisyphus/evidence/task-6-syntax-valid.txt

  Scenario: Launch file structure validation
    Tool: Bash
    Preconditions: Launch file exists
    Steps:
      1. grep -c "IncludeLaunchDescription" launch/real_bringup.launch.py
      2. grep -c "turtlebot3_bringup" launch/real_bringup.launch.py
      3. grep -c "nav2_bringup" launch/real_bringup.launch.py
      4. grep -c "slam_toolbox" launch/real_bringup.launch.py
    Expected Result:
      - At least 3 IncludeLaunchDescription calls
      - turtlebot3_bringup referenced (robot drivers)
      - nav2_bringup referenced
      - slam_toolbox referenced
    Failure Indicators:
      - Missing subsystems
      - Fewer than 3 includes
    Evidence: .sisyphus/evidence/task-6-structure.txt
  ```

  **Evidence to Capture**:
  - [ ] task-6-syntax-valid.txt (Python compilation + use_sim_time check)
  - [ ] task-6-structure.txt (launch file structure validation)

  **Commit**: YES
  - Message: `feat(robot): add real robot launch file`
  - Files: `launch/real_bringup.launch.py`
  - Pre-commit: `python3 -m py_compile launch/real_bringup.launch.py`

- [x] 7. Nav2 Parameters Configuration

  **What to do**:
  - Create `config/nav2_params.yaml` with Nav2 stack parameters
  - Start with conservative values (low max velocity, high obstacle inflation)
  - Key parameters:
    - Robot footprint (TurtleBot3 Burger dimensions)
    - Max velocities (linear: 0.22 m/s, angular: 2.84 rad/s)
    - Obstacle inflation radius (0.3m for safety)
    - Costmap resolution (0.05m)
    - Path planning algorithm (default DWB)
  - Same config used for both sim and real robot
  - Add comments explaining critical parameters

  **Must NOT do**:
  - Create separate configs for sim/real
  - Add custom planners or controllers
  - Over-tune for perfect performance (accept "good enough")

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Standard Nav2 config with documented defaults
  - **Skills**: []
    - No specialized skills needed

  **Parallelization**:
  - **Can Run In Parallel**: YES (after Task 1)
  - **Parallel Group**: Wave 2 (with Tasks 4, 5, 6, 8)
  - **Blocks**: Tasks 5, 6 (launch files need this config)
  - **Blocked By**: Task 1 (needs config/ directory)

  **References**:
  - Pattern: `nav2_bringup/params/nav2_params.yaml` - Default Nav2 parameters
  - Official docs: https://navigation.ros.org/configuration/index.html - Nav2 parameter reference
  - TurtleBot3 specs: Max linear velocity 0.22 m/s, max angular velocity 2.84 rad/s

  **Acceptance Criteria**:
  - [ ] Config file created at `config/nav2_params.yaml`
  - [ ] Robot footprint defined (TurtleBot3 Burger dimensions)
  - [ ] Max velocities set (linear and angular)
  - [ ] Costmap parameters configured
  - [ ] YAML syntax valid

  **QA Scenarios (MANDATORY)**:

  ```
  Scenario: YAML syntax validation
    Tool: Bash
    Preconditions: Config file exists
    Steps:
      1. python3 -c "import yaml; yaml.safe_load(open('config/nav2_params.yaml'))"
      2. echo $?
    Expected Result:
      - Exit code 0 (valid YAML)
      - No parse errors
    Failure Indicators:
      - Non-zero exit code
      - YAML syntax error
    Evidence: .sisyphus/evidence/task-7-yaml-valid.txt

  Scenario: Critical parameters present
    Tool: Bash
    Preconditions: Config file exists
    Steps:
      1. grep -E "(robot_radius|footprint)" config/nav2_params.yaml
      2. grep -E "max_vel" config/nav2_params.yaml
      3. grep -E "inflation_radius" config/nav2_params.yaml
      4. grep -E "resolution" config/nav2_params.yaml
    Expected Result:
      - Robot footprint or radius defined
      - Max velocity parameters present
      - Inflation radius configured
      - Costmap resolution set
    Failure Indicators:
      - Missing critical parameters
      - Empty grep results
    Evidence: .sisyphus/evidence/task-7-params-check.txt
  ```

  **Evidence to Capture**:
  - [ ] task-7-yaml-valid.txt (YAML validation)
  - [ ] task-7-params-check.txt (parameter presence check)

  **Commit**: YES
  - Message: `feat(config): add Nav2 parameters`
  - Files: `config/nav2_params.yaml`
  - Pre-commit: `python3 -c "import yaml; yaml.safe_load(open('config/nav2_params.yaml'))"`

- [x] 8. RViz Configuration

  **What to do**:
  - Create `rviz/nav.rviz` RViz configuration file
  - Display components:
    - Robot model (TurtleBot3 URDF)
    - LIDAR scan (/scan topic)
    - Map (/map topic from SLAM)
    - Global costmap
    - Local costmap
    - Global path
    - Local path
    - Robot footprint
    - Zone markers (MarkerArray on /zone_markers topic - placeholder for future)
  - Set fixed frame to "map"
  - Reasonable camera position (top-down view)

  **Must NOT do**:
  - Add custom visualization plugins
  - Create multiple RViz configs
  - Add web dashboard or GUI

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Standard RViz config with common displays
  - **Skills**: []
    - No specialized skills needed

  **Parallelization**:
  - **Can Run In Parallel**: YES (after Task 1)
  - **Parallel Group**: Wave 2 (with Tasks 4, 5, 6, 7)
  - **Blocks**: Tasks 10, 11 (tests and docs reference RViz)
  - **Blocked By**: Task 1 (needs rviz/ directory)

  **References**:
  - Pattern: `nav2_bringup/rviz/nav2_default_view.rviz` - Nav2 default RViz config
  - Pattern: `turtlebot3_gazebo/rviz/tb3_gazebo.rviz` - TurtleBot3 RViz setup

  **Acceptance Criteria**:
  - [ ] RViz config created at `rviz/nav.rviz`
  - [ ] Fixed frame set to "map"
  - [ ] Robot model display configured
  - [ ] Map, costmaps, paths displays configured
  - [ ] LIDAR scan display configured

  **QA Scenarios (MANDATORY)**:

  ```
  Scenario: RViz config loads without errors
    Tool: Bash
    Preconditions: RViz config exists, sim launch running
    Steps:
      1. source install/setup.bash
      2. timeout 60s ros2 launch nutrient_nexus_navigation sim_bringup.launch.py &
      3. sleep 30
      4. timeout 20s rviz2 -d rviz/nav.rviz 2>&1 | tee /tmp/rviz_output.txt &
      5. sleep 10
      6. pkill -9 rviz2
      7. pkill -9 -f "ros2 launch"
      8. pkill -9 gazebo
      9. grep -i "error" /tmp/rviz_output.txt
    Expected Result:
      - RViz starts without errors
      - No "error" in output
    Failure Indicators:
      - RViz crashes
      - Error messages in output
    Evidence: .sisyphus/evidence/task-8-rviz-load.txt

  Scenario: RViz displays configured
    Tool: Bash
    Preconditions: RViz config exists
    Steps:
      1. grep -c "RobotModel" rviz/nav.rviz
      2. grep -c "LaserScan" rviz/nav.rviz
      3. grep -c "Map" rviz/nav.rviz
      4. grep -c "Path" rviz/nav.rviz
    Expected Result:
      - RobotModel display present (count >= 1)
      - LaserScan display present (count >= 1)
      - Map display present (count >= 1)
      - Path displays present (count >= 2, global + local)
    Failure Indicators:
      - Missing displays
      - Zero counts
    Evidence: .sisyphus/evidence/task-8-displays.txt
  ```

  **Evidence to Capture**:
  - [ ] task-8-rviz-load.txt (RViz startup output)
  - [ ] task-8-displays.txt (display configuration check)

  **Commit**: YES
  - Message: `feat(viz): add RViz configuration`
  - Files: `rviz/nav.rviz`
  - Pre-commit: `grep "Fixed Frame" rviz/nav.rviz` (manual check)

- [x] 9. Unit Tests for Zone Detection

  **What to do**:
  - Create `test/test_zone_detection.py` with pytest
  - Test zone detection logic with mock positions:
    - Position inside zone_a → returns "zone_a"
    - Position inside zone_b → returns "zone_b"
    - Position inside zone_c → returns "zone_c"
    - Position outside all zones → returns "no_zone"
    - Position on zone boundary → returns zone name (boundary inclusive)
  - Mock YAML loading (don't require actual file)
  - Test coordinate comparison logic directly

  **Must NOT do**:
  - Test ROS2 node integration (that's Task 10)
  - Add complex test fixtures or factories
  - Test SLAM or Nav2 (out of scope)

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Standard pytest unit tests with simple logic
  - **Skills**: []
    - No specialized skills needed

  **Parallelization**:
  - **Can Run In Parallel**: YES (after Task 4)
  - **Parallel Group**: Wave 3 (with Tasks 10, 11, 12)
  - **Blocks**: None (tests don't block other work)
  - **Blocked By**: Task 4 (needs zone detector code)

  **References**:
  - Pattern: ROS2 Python packages use pytest for unit tests
  - API: `pytest.fixture` for test setup
  - Pattern: Mock YAML data with dictionaries

  **Acceptance Criteria**:
  - [ ] Test file created at `test/test_zone_detection.py`
  - [ ] At least 5 test cases (3 zones + outside + boundary)
  - [ ] `pytest test/test_zone_detection.py -v` passes all tests
  - [ ] Tests run in < 5 seconds

  **QA Scenarios (MANDATORY)**:

  ```
  Scenario: Unit tests pass
    Tool: Bash
    Preconditions: Test file exists, package built
    Steps:
      1. source install/setup.bash
      2. pytest test/test_zone_detection.py -v --tb=short 2>&1 | tee /tmp/pytest_output.txt
      3. echo "Exit code: $?"
      4. grep -E "(PASSED|FAILED)" /tmp/pytest_output.txt
    Expected Result:
      - Exit code 0
      - All tests show "PASSED"
      - No "FAILED" tests
    Failure Indicators:
      - Non-zero exit code
      - Any test failures
    Evidence: .sisyphus/evidence/task-9-pytest.txt

  Scenario: Test coverage validation
    Tool: Bash
    Preconditions: Test file exists
    Steps:
      1. grep -c "def test_" test/test_zone_detection.py
      2. grep -E "(zone_a|zone_b|zone_c|no_zone)" test/test_zone_detection.py
    Expected Result:
      - At least 5 test functions
      - All zone names tested (zone_a, zone_b, zone_c, no_zone)
    Failure Indicators:
      - Fewer than 5 tests
      - Missing zone coverage
    Evidence: .sisyphus/evidence/task-9-coverage.txt
  ```

  **Evidence to Capture**:
  - [ ] task-9-pytest.txt (pytest output)
  - [ ] task-9-coverage.txt (test count and coverage)

  **Commit**: YES
  - Message: `test(nav): add zone detection unit tests`
  - Files: `test/test_zone_detection.py`
  - Pre-commit: `pytest test/test_zone_detection.py -v`

- [x] 10. Integration Test for Simulation Navigation

  **What to do**:
  - Create `test/test_sim_navigation.py` using launch_testing
  - In this environment, implement the strongest honest automated launch/integration check that can be proven reliably
  - Test workflow:
    1. Launch sim_bringup.launch.py
    2. Wait for nodes/topics to be ready (at minimum check `/scan`, `/clock`, and `zone_detector`)
    3. Verify the launched system remains healthy through readiness checks
    4. Shutdown cleanly and assert no early process death or traceback
  - Use launch_testing framework (not manual bash scripts)
  - Add retry logic for node readiness checks
  - If full Nav2 goal execution becomes reliable later, extend this test rather than faking success

  **Must NOT do**:
  - Test real robot (that's Task 12)
  - Test perfect navigation (accept "close enough")
  - Add custom Nav2 behaviors

  **Recommended Agent Profile**:
  - **Category**: `unspecified-high`
    - Reason: Complex integration test with timing, launch_testing API
  - **Skills**: []
    - No specialized skills needed, but higher complexity

  **Parallelization**:
  - **Can Run In Parallel**: YES (after Tasks 5, 8)
  - **Parallel Group**: Wave 3 (with Tasks 9, 11, 12)
  - **Blocks**: None
  - **Blocked By**: Tasks 5 (sim launch), 8 (RViz config for reference)

  **References**:
  - Pattern: `nav2_bringup/test/` - Nav2 integration tests with launch_testing
  - API: `launch_testing.actions.ReadyToTest` for node readiness
  - API: `rclpy.action.ActionClient` for sending nav goals
  - Pattern: Use `ros2 topic hz` to check topic readiness

  **Acceptance Criteria**:
  - [ ] Test file created at `test/test_sim_navigation.py`
  - [ ] Test launches sim_bringup.launch.py
  - [ ] Test waits for node readiness
  - [ ] Test verifies launch readiness and clean shutdown honestly
  - [ ] `pytest test/test_sim_navigation.py -v` passes

  **QA Scenarios (MANDATORY)**:

  ```
  Scenario: Integration test passes
    Tool: Bash
    Preconditions: Package built, all dependencies installed
    Steps:
      1. source install/setup.bash
      2. pytest test/test_sim_navigation.py -v --tb=short 2>&1 | tee /tmp/integration_test.txt
      3. echo "Exit code: $?"
      4. grep -E "(PASSED|FAILED)" /tmp/integration_test.txt
    Expected Result:
      - Exit code 0
      - Test shows "PASSED"
      - No "FAILED"
    Failure Indicators:
      - Non-zero exit code
      - Test failure
      - Timeout errors
    Evidence: .sisyphus/evidence/task-10-integration.txt

  Scenario: Test launches and cleans up
    Tool: Bash
    Preconditions: Test file exists
    Steps:
      1. grep -c "launch_testing" test/test_sim_navigation.py
      2. grep -c "ReadyToTest" test/test_sim_navigation.py
      3. grep -c "shutdown" test/test_sim_navigation.py
    Expected Result:
      - launch_testing imported
      - ReadyToTest used for synchronization
      - Proper shutdown logic present
    Failure Indicators:
      - Missing launch_testing
      - No readiness checks
      - No cleanup
    Evidence: .sisyphus/evidence/task-10-structure.txt
  ```

  **Evidence to Capture**:
  - [ ] task-10-integration.txt (integration test output)
  - [ ] task-10-structure.txt (test structure validation)

  **Commit**: YES
  - Message: `test(sim): add simulation integration test`
  - Files: `test/test_sim_navigation.py`
  - Pre-commit: `pytest test/test_sim_navigation.py -v` (may take 60s+)

- [x] 11. README Documentation

  **What to do**:
  - Create comprehensive `README.md` with sections:
    1. **Project Overview**: NutrientNexus Week 1 navigation foundation
    2. **Prerequisites**: ROS2 Jazzy, TurtleBot3 packages, Gazebo
    3. **Installation**: Build instructions with colcon
    4. **Configuration**: Explain zones.yaml format and nav2_params.yaml
    5. **Usage - Simulation**: How to launch sim_bringup.launch.py
    6. **Usage - Real Robot**: How to launch real_bringup.launch.py
    7. **Testing**: How to run unit and integration tests
    8. **Topics**: List of published/subscribed topics
    9. **Troubleshooting**: Common issues (SLAM fails, Gazebo crashes, network setup)
    10. **Architecture**: Brief overview of nodes and data flow
  - Include example commands with expected outputs
  - Add troubleshooting section for common issues

  **Must NOT do**:
  - Write extensive API documentation (code comments sufficient)
  - Create separate documentation files
  - Add tutorials beyond basic usage

  **Recommended Agent Profile**:
  - **Category**: `writing`
    - Reason: Documentation task, requires clear technical writing
  - **Skills**: []
    - No specialized skills needed

  **Parallelization**:
  - **Can Run In Parallel**: YES (after Task 8)
  - **Parallel Group**: Wave 3 (with Tasks 9, 10, 12)
  - **Blocks**: None
  - **Blocked By**: Task 8 (needs RViz config to document)

  **References**:
  - Pattern: ROS2 package READMEs typically include installation, usage, topics
  - Example: TurtleBot3 documentation structure

  **Acceptance Criteria**:
  - [ ] README.md created at project root
  - [ ] All 10 sections present
  - [ ] Example commands included with expected outputs
  - [ ] Topics table lists /current_zone, /odom, /scan, /map, /cmd_vel
  - [ ] Troubleshooting section has at least 3 common issues

  **QA Scenarios (MANDATORY)**:

  ```
  Scenario: README completeness check
    Tool: Bash
    Preconditions: README.md exists
    Steps:
      1. grep -c "## " README.md
      2. grep -E "(Installation|Usage|Testing|Topics|Troubleshooting)" README.md
      3. grep -c "ros2 launch" README.md
      4. grep -c "/current_zone" README.md
    Expected Result:
      - At least 10 section headers (##)
      - All required sections present
      - At least 3 ros2 launch commands
      - /current_zone topic documented
    Failure Indicators:
      - Fewer than 10 sections
      - Missing required sections
      - No example commands
    Evidence: .sisyphus/evidence/task-11-readme-check.txt

  Scenario: Markdown syntax validation
    Tool: Bash
    Preconditions: README.md exists
    Steps:
      1. python3 -c "import markdown; markdown.markdown(open('README.md').read())"
      2. echo $?
    Expected Result:
      - Exit code 0 (valid Markdown)
      - No parse errors
    Failure Indicators:
      - Non-zero exit code
      - Markdown syntax errors
    Evidence: .sisyphus/evidence/task-11-markdown-valid.txt
  ```

  **Evidence to Capture**:
  - [ ] task-11-readme-check.txt (completeness validation)
  - [ ] task-11-markdown-valid.txt (Markdown syntax check)

  **Commit**: YES
  - Message: `docs: add README with setup instructions`
  - Files: `README.md`
  - Pre-commit: `grep "## " README.md | wc -l` (check section count)

- [x] 12. Real Robot Validation (MANUAL - Physical Hardware Required)

  **What to do**:
  - Manual validation on physical TurtleBot3 Burger (requires human operator)
  - Suggested validation workflow (adapt as needed based on available hardware setup):
    1. Connect to TurtleBot3 (SSH, direct terminal, or other access method)
    2. Launch real_bringup.launch.py
    3. Verify all nodes start without errors (check terminal output)
    4. Check /scan topic publishes at ~5 Hz: `ros2 topic hz /scan`
    5. Test basic movement (teleop keyboard, RViz nav goals, or other method)
    6. Verify SLAM builds map: `ros2 topic echo /map --once`
    7. Attempt autonomous navigation (method flexible: RViz, command line, etc.)
    8. Check /current_zone publishes when robot moves: `ros2 topic echo /current_zone`
  - Document any issues found (acceptable for Week 1)
  - Capture evidence: terminal logs, RViz screenshots, video of robot movement (methods flexible)
  - **Note**: This task requires physical hardware and human interaction - cannot be fully automated. Specific methods (SSH vs direct, teleop vs other control) are suggestions, not requirements.

  **Must NOT do**:
  - Require perfect navigation (tuning deferred)
  - Add custom recovery behaviors
  - Test in production environment

  **Recommended Agent Profile**:
  - **Category**: `unspecified-high`
    - Reason: Real hardware validation, requires careful testing
  - **Skills**: []
    - No specialized skills needed

  **Parallelization**:
  - **Can Run In Parallel**: YES (after Task 6)
  - **Parallel Group**: Wave 3 (with Tasks 9, 10, 11)
  - **Blocks**: None
  - **Blocked By**: Task 6 (needs real robot launch file)

  **References**:
  - Pattern: TurtleBot3 bringup documentation for SSH and network setup
  - Troubleshooting: ROS_DOMAIN_ID, network configuration

  **Acceptance Criteria**:
  - [ ] real_bringup.launch.py starts without errors on real robot (human verifies terminal output)
  - [ ] /scan topic publishes at expected rate: `ros2 topic hz /scan` shows ~5 Hz (human runs command)
  - [ ] Robot responds to movement commands (method flexible: teleop, nav goals, etc.)
  - [ ] SLAM publishes map data: `ros2 topic echo /map --once` returns data (human runs command)
  - [ ] Robot attempts autonomous navigation (method flexible, success not required for Week 1)
  - [ ] Evidence captured: terminal logs saved, RViz screenshot taken, video recorded (human captures)

  **QA Scenarios (MANUAL - Human Operator Required)**:

  ```
  Scenario: Real robot bringup (MANUAL - suggested workflow)
    Tool: Bash (human operator on TurtleBot3 or via SSH)
    Preconditions: TurtleBot3 powered on, network configured, human has access
    Steps (adapt as needed for your setup):
      1. Human connects to robot (SSH ubuntu@{IP} or direct terminal)
      2. Human sources ROS2: source /opt/ros/jazzy/setup.bash
      3. Human sources workspace: source ~/workspace/install/setup.bash
      4. Human launches: ros2 launch nutrient_nexus_navigation real_bringup.launch.py 2>&1 | tee /tmp/real_launch.txt &
      5. Human waits: sleep 10
      6. Human checks nodes: ros2 node list > /tmp/node_list.txt
      7. Human checks scan rate: ros2 topic hz /scan --window 10 2>&1 | head -20 > /tmp/scan_rate.txt
      8. Human stops: pkill -9 -f "ros2 launch"
    Expected Result:
      - Launch completes without errors (human reads terminal)
      - All nodes present in node list (human inspects file)
      - /scan publishes at ~5 Hz (human reads output)
    Failure Indicators:
      - Launch errors (human observes)
      - Missing nodes (human checks list)
      - No /scan messages (human verifies)
    Evidence: .sisyphus/evidence/task-12-real-bringup.txt (human saves)

  Scenario: Movement test (MANUAL - suggested workflow)
    Tool: Bash + control method of choice (human operator required)
    Preconditions: real_bringup.launch.py running on robot, human at laptop
    Steps (example using teleop - adapt as needed):
      1. Human sources: source install/setup.bash
      2. Human runs control method (e.g., ros2 run turtlebot3_teleop teleop_keyboard)
      3. Human commands robot to move forward
      4. Human observes robot movement (visual confirmation)
      5. Human commands robot to stop
      6. Human records video of movement
    Expected Result:
      - Robot moves when commanded (human observes)
      - Robot stops when commanded (human observes)
      - No crashes or errors (human monitors terminal)
    Failure Indicators:
      - Robot doesn't move (human observes)
      - Connection errors (human reads terminal)
    Evidence: .sisyphus/evidence/task-12-movement-video.mp4 (human records)
  ```

  **Evidence to Capture**:
  - [ ] task-12-real-bringup.txt (launch output + node/topic checks)
  - [ ] task-12-movement-video.mp4 (video of robot moving)

  **Commit**: NO (validation only, no code changes)

---

## Final Verification Wave (MANDATORY — after ALL implementation tasks)

> 4 review agents run in PARALLEL. ALL must APPROVE. Present consolidated results to user and get explicit "okay" before completing.
>
> **Do NOT auto-proceed after verification. Wait for user's explicit approval before marking work complete.**
> **Never mark F1-F4 as checked before getting user's okay.** Rejection or user feedback -> fix -> re-run -> present again -> wait for okay.

- [x] F1. **Plan Compliance Audit** — `oracle`
  Read the plan end-to-end. For each "Must Have": verify implementation exists (read file, curl endpoint, run command). For each "Must NOT Have": search codebase for forbidden patterns — reject with file:line if found. Check evidence files exist in .sisyphus/evidence/. Compare deliverables against plan.
  Output: `Must Have [N/N] | Must NOT Have [N/N] | Tasks [N/N] | VERDICT: APPROVE/REJECT`

- [x] F2. **Code Quality Review** — `unspecified-high`
  Run linter + `pytest test/`. Review all changed files for: commented-out code, unused imports, hardcoded paths. Check AI slop: excessive comments, over-abstraction, generic names (data/result/item/temp).
  Output: `Lint [PASS/FAIL] | Tests [N pass/N fail] | Files [N clean/N issues] | VERDICT`

- [x] F3. **Real Manual QA (MANUAL - Physical Hardware Required)** — `unspecified-high`
  Start from clean state. Execute QA scenarios from Tasks 1-11 (automated). For Task 12 scenarios: human operator performs real robot validation following documented steps. Test cross-task integration (navigation + zone detection working together in sim). Test edge cases in simulation: robot outside zones, SLAM initialization, obstacle avoidance. Save to `.sisyphus/evidence/final-qa/`. Real robot testing requires physical hardware and human interaction.
  Output: `Scenarios [N/N pass] | Integration [N/N] | Edge Cases [N tested] | VERDICT`

- [x] F4. **Scope Fidelity Check** — `deep`
  For each task: read "What to do", read actual diff (git log/diff). Verify 1:1 — everything in spec was built (no missing), nothing beyond spec was built (no creep). Check "Must NOT do" compliance. Detect cross-task contamination: Task N touching Task M's files. Flag unaccounted changes.
  Output: `Tasks [N/N compliant] | Contamination [CLEAN/N issues] | Unaccounted [CLEAN/N files] | VERDICT`

---

## Commit Strategy

- **After Task 1**: `feat(setup): add ROS2 package scaffolding` - setup.py, package.xml, directory structure
- **After Task 2**: `feat(sim): add Gazebo farm box world` - worlds/farm_box.world
- **After Task 3**: `feat(config): add zone definitions` - config/zones.yaml
- **After Task 4**: `feat(nav): add zone detector node` - zone_detector.py
- **After Task 5**: `feat(sim): add simulation launch file` - sim_bringup.launch.py
- **After Task 6**: `feat(robot): add real robot launch file` - real_bringup.launch.py
- **After Task 7**: `feat(config): add Nav2 parameters` - config/nav2_params.yaml
- **After Task 8**: `feat(viz): add RViz configuration` - rviz/nav.rviz
- **After Task 9**: `test(nav): add zone detection unit tests` - test/test_zone_detection.py
- **After Task 10**: `test(sim): add simulation integration test` - test/test_sim_navigation.py
- **After Task 11**: `docs: add README with setup instructions` - README.md
- **After Task 12**: `test(robot): validate real robot bringup` - (no files, validation only)

---

## Success Criteria

### Verification Commands
```bash
# Simulation test
ros2 launch nutrient_nexus_navigation sim_bringup.launch.py
# Expected: Gazebo opens, robot spawns, SLAM starts, no errors

# Zone detection test
ros2 topic echo /current_zone --once
# Expected: Publishes zone name (zone_a, zone_b, zone_c, or no_zone)

# Unit tests
pytest test/test_zone_detection.py -v
# Expected: All tests pass

# Integration test
pytest test/test_sim_navigation.py -v
# Expected: Launch/readiness integration test passes without early process death or traceback

# Real robot test
ros2 launch nutrient_nexus_navigation real_bringup.launch.py
# Expected: Launches without errors, robot responds to teleop
```

### Final Checklist
- [ ] All "Must Have" present (verified by F1)
- [ ] All "Must NOT Have" absent (verified by F1)
- [ ] All tests pass (verified by F2)
- [ ] All QA scenarios executed with evidence (verified by F3)
- [ ] No scope creep detected (verified by F4)
- [ ] User explicitly approves final verification results
