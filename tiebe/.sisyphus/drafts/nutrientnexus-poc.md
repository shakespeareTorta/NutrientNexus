# Draft: NutrientNexus PoC - ROS2 + Gazebo Implementation

## Core Objective (Week 1 - Phase 1)

**Single objective**: Build a ROS2 Python navigation package that enables TurtleBot3 Burger to autonomously map and navigate a 4m x 4m environment with crop zones, working in both Gazebo simulation and on the real robot.

## Project Overview (from presentation)

**System Name**: NutrientNexus  
**Purpose**: Digital Twin Autonomous System for Agricultural and Coastal Field Monitoring and Decision Making

**Core Concept**: Autonomous field monitoring and decision-support system that integrates farm and coastal environments to optimize crop management while minimizing environmental impact (eutrophication).

## PoC Scope (from presentation slides 155-223)

**Focus**: Crop robots only (not coastal monitoring for PoC)

**Physical Twin** (TurtleBot3 Burger) collects:
- Soil water/nutrient level (STUBBED - no actual sensors)
- Past crop growth history (STUBBED)

**Digital Twin** integrates:
- Weather + water cycle data
- Discharge thresholds
- Per-crop nutrient/irrigation needs

**Output**: DT computes optimal fertilizer and irrigation amounts, balancing:
1. High growth rate
2. Environmental impact

## Technical Constraints

**Hardware**: TurtleBot3 Burger with LIDAR only
- Cannot add additional sensors
- Must stub out: soil moisture sensor, nutrient sensor, growth monitoring

**Software Stack**:
- ROS2 Jazzy (already configured in flake.nix)
- Gazebo simulation
- Nix flake environment (already working)

## Functional Requirements (from slide 197-202)

**Functional**:
- Map + navigate crop zones
- Digital twin integrate with external data
- Optimization logic balances yield and environment

**Non-functional**:
- Synchronization between physical and digital twins
- Avoid danger zones
- Generate concrete recommendations/decisions

## Demonstration Goals (slide 214-222)

- Field map with crop zones
- Report (simulated) moisture/nutrient/growth levels
- Show one zone with fertilizer advice, another with irrigation advice
- High-runoff-risk situation where fertilizer is reduced/postponed
- Live updates on digital twin during scanning

## Current State

- Nix flake with ROS2 Jazzy + Gazebo is configured
- TurtleBot3 packages available: msgs, description, simulations, gazebo, navigation2, teleop
- No ROS2 workspace or packages created yet

## Open Questions

### Architecture & System Design
1. **Concept Map**: I couldn't extract the cmap.pdf - can you describe the key components and their relationships?

### ROS2 Architecture
2. **Node Structure**: How should we organize the ROS2 nodes?
   - Separate nodes for: robot control, sensor stubbing, digital twin logic, decision engine?
   - Or monolithic approach?

3. **Communication Pattern**: 
   - Should sensor data be published on topics, or use services?
   - How should digital twin and physical twin synchronize?

### Simulation Environment
4. **Gazebo World**: 
   - Do you want a custom field environment with crop zones marked?
   - Or use a simple world with designated areas?
   - How many crop zones? (2-3 for demo?)

5. **Crop Zone Representation**:
   - Visual markers in Gazebo?
   - Coordinate-based zones?
   - How should robot know it entered a zone?

### Sensor Stubbing Strategy
6. **Stub Implementation**:
   - Random values within realistic ranges?
   - Predefined values per zone (zone A = low moisture, zone B = high nutrients)?
   - Time-varying to show "scanning" behavior?

7. **Sensor Types to Stub**:
   - Soil moisture (0-100%?)
   - Nutrient levels (N, P, K in ppm?)
   - Crop growth metrics (height, health index?)

### Digital Twin & Decision Logic
8. **External Data Integration**:
   - Mock weather data, or integrate real API?
   - Discharge thresholds - hardcoded or configurable?
   - Crop nutrient needs - lookup table?

9. **Optimization Algorithm**:
   - Simple rule-based (if moisture < X, irrigate Y)?
   - Or more sophisticated scoring system?
   - How to represent "environmental impact" quantitatively?

10. **Decision Output Format**:
    - Text recommendations?
    - Visualization in RViz?
    - Log files?

### Navigation & Mapping
11. **Navigation Strategy**:
    - Pre-defined waypoints per zone?
    - Autonomous exploration with SLAM?
    - Teleoperation for demo?

12. **Mapping Requirements**:
    - Use Nav2 for mapping?
    - Or just localization in known map?

### Testing & Verification
13. **Test Scenarios**:
    - Which specific scenarios to implement for demo?
    - Scenario 1: Low moisture → irrigation advice?
    - Scenario 2: High nutrients + rain forecast → delay fertilizer?

14. **Success Criteria**:
    - What does "working PoC" look like to you?
    - Robot navigates zones + displays recommendations?
    - Or more sophisticated integration?

### Development Priorities
15. **MVP Definition**:
    - What's the absolute minimum for a viable demo?
    - What can be simplified or deferred?

16. **Timeline**:
    - When do you need this working?
    - Any intermediate milestones?

## System Architecture (from cmaps.txt)

### Level 0: High-Level System
**Digital Twin for Predicting and Reducing Farm-to-Coast Nutrient Pollution**

**Inputs**: Yield targets, field boundaries, weather data, crop database, regulatory limits
**Outputs**: Nitrogen application recommendations, runoff risk warnings, sustainability reports

### Level 1: Component Architecture

**Core Components**:
1. **Decision Intelligence** - Makes decisions balancing yield vs environmental impact
2. **Digital Twin Models** - Simulates water cycle, nutrient availability, crop growth
3. **Physical Sensing & Actuation** - Robot sensors + actuators (fertilizer/irrigation)
4. **Historical Field & Twin Database** - Stores sensor data, decisions, outcomes
5. **External Data Integration** - Weather API, crop database
6. **Stakeholder Interfaces** - Farmer, regulators, maintenance

**Key Data Flows**:
- Sensors → Digital Twin → Decision Intelligence → Actuator Commands
- Digital Twin runs: water cycle model, nutrient model, crop growth history, contamination thresholds
- Decision Intelligence outputs: fertilizer amounts, irrigation schedules, runoff warnings

### Physical Twin (TurtleBot3 for PoC)
**Sensors** (to be stubbed):
- Soil moisture
- Soil nutrients (N, P, K)
- Thermal (temperature)
- Plant transpiration (indirectly: growth/health)

**Actuators** (simulated for PoC):
- Precision fertilizer applicator
- Irrigation sprinkler

## User Requirements (confirmed)

### Primary Goals
1. **Framework for entire project** - ROS2 package structure for all components
2. **Navigation implemented** - Robot can map and navigate crop zones autonomously

### Sensor Stubbing Strategy (confirmed)
- **Smart stubbing**: Not random, but logically changing over time
- **Memory per area**: Each crop zone remembers its state
- **Time-varying**: Values change based on simulated conditions (weather, time since last irrigation/fertilization)

### Physical Environment (confirmed)
- **Size**: Large wooden box, approximately 4m x 4m
- **Obstacles**: Walls around perimeter + single obstacle in middle
- **Navigation**: Must have obstacle detection (LIDAR-based)
- **Crop zones**: 2-3 designated coordinate-based zones

### Digital Twin Communication (from slides)
**From slides 208, 221**: "Synchronisation between physical and digital twins" + "Live updates on digital twin during scanning"

**Interpretation**: **Scenario A - Simulation mirrors real robot**
- Physical twin (robot) collects: soil water/nutrient level, crop growth history
- Digital twin integrates: weather + water cycle, discharge thresholds, per-crop needs
- Digital twin computes: optimal fertilizer and irrigation amounts
- **Bidirectional flow**:
  - Physical → Digital: Sensor readings (stubbed), position, zone data
  - Digital → Physical: Recommendations, decisions, actuator commands
- **Live synchronization**: Digital twin updates in real-time as robot scans

### RViz Visualization (explanation provided)
**RViz** = ROS Visualization tool. Shows:
- Robot model in 3D
- Sensor data (LIDAR scans, camera feeds)
- Navigation paths, waypoints, costmaps
- Custom markers (crop zones, sensor readings, recommendations)

For PoC: Use RViz to display robot position, LIDAR, navigation, and overlay crop zone data + recommendations

## Technical Decisions (CONFIRMED)

### 1. Package Structure
**Monolithic Python package**: `nutrient_nexus_navigation`
- Single ROS2 Python package (Metis directive)
- All nodes in one package for simplicity
- Easier to manage for PoC phase

### 2. Navigation Approach
**Full SLAM + Autonomous Navigation (Option B)**
- Robot builds map using LIDAR (SLAM)
- Nav2 stack for path planning and obstacle avoidance
- Autonomous navigation to crop zones
- Most realistic for final deployment

### 3. Crop Zone Definition
**Axis-aligned bounding boxes (Metis directive)**
- Define zones as rectangles in YAML config
- Format: `zone_a: {min_x: 0.5, max_x: 1.5, min_y: 0.5, max_y: 1.5}`
- Robot detects zone by simple coordinate check
- Simple, no polygon math needed
- Outside all zones → publish "no_zone"

### 4. Sensor Stub Logic (CONFIRMED)
**Smart, stateful stubbing**:

**Soil Moisture** (0-100%):
- Initial per zone: A=30%, B=60%, C=45%
- Decreases: -2% per simulated hour
- Irrigation: +30%
- Rain: +10%

**Soil Nutrients** (N, P, K in ppm):
- Initial per zone: A=low N, B=high P, C=balanced
- Consumption: -5 ppm per simulated day
- Fertilization: +50 ppm
- Rain leaching: -10 ppm

**Crop Health** (0-100 index):
- Initial: 70% per zone
- Optimal conditions: +5% per day
- Deficiencies: -10% per day

### 5. Decision Engine
**Scoring system (Option B)**:
- Calculate yield score (moisture, nutrients, health)
- Calculate environmental score (runoff risk, contamination)
- Optimize: 0.6 * yield + 0.4 * environment
- More sophisticated than simple rules

### 6. External Data Integration
**Weather**: Mock/hardcoded (rain forecast = 5mm tomorrow)
**Crop Database**: JSON config file with crop requirements

### 7. Demo Scenario (CONFIRMED)
**Initial State**:
- Zone A: Low moisture (30%) → Irrigation needed
- Zone B: Low nitrogen (40ppm) → Fertilizer needed
- Zone C: Optimal → No action

**Event**: Rain forecast (15mm in 6 hours)

**Updated Decisions**:
- Zone A: Irrigation postponed
- Zone B: Fertilizer postponed (runoff risk)
- Zone C: No change

**Robot Actions**:
1. Navigate to each zone
2. Scan sensors (stubbed)
3. Display recommendations in RViz
4. Weather event → real-time updates

### 8. Timeline & Scope
**Week 1 Focus** (THIS PLAN):
- **Option A - Navigation only** (CONFIRMED)
  - ✓ ROS2 workspace + package structure (framework ready)
  - ✓ Gazebo world (4m x 4m box with walls + center obstacle)
  - ✓ TurtleBot3 Burger spawned in Gazebo
  - ✓ SLAM Toolbox mapping
  - ✓ Nav2 autonomous navigation with obstacle avoidance
  - ✓ Coordinate-based crop zone definitions (config file)
  - ✓ Zone detection (robot knows which zone it's in)
  - ✓ Same code works on real robot (sim-to-real ready)
  - ✓ RViz visualization with zone markers
  - ✓ Digital twin bidirectional communication **framework** (topics/interfaces defined, ready for sensor data)
  - ✗ Sensor stubbing (deferred to Week 2+)
  - ✗ Decision engine (deferred to Week 2+)

**Architecture prepared for future weeks**:
- Sensor stub nodes (skeleton ready, logic deferred)
- Decision engine node (skeleton ready, logic deferred)
- Digital twin state management (framework ready)
- Report generation (framework ready)

**Out of scope for Week 1**:
- Full sensor stubbing logic (basic version only)
- Complete decision engine (basic version only)
- Advanced scenarios

### 9. Testing Strategy
**Test approach**: Tests-after + Agent QA
- Write implementation first, then tests
- Unit tests: pytest for zone detection logic, sensor stubs (deferred)
- Integration tests: launch_testing for sim navigation
- **Agent QA (MANDATORY)**: Every task includes agent-executable QA scenarios
  - Frontend/UI: Playwright (not applicable for Week 1)
  - CLI/TUI: interactive_bash with tmux
  - ROS nodes: Bash with ros2 commands (topic echo, node list, launch)
  - Evidence saved to `.sisyphus/evidence/`
- No "user manually tests" acceptance criteria allowed

### 10. Deployment Target
**Simulation + Physical Robot**:
- Develop in Gazebo simulation
- Deploy to real TurtleBot3 Burger for testing
- Must work on both without code changes

### 11. Logging
**ROS logs**: Standard ROS logging
**Reports**: Separate files for recommendations/decisions

### 12. Configuration
**YAML files** for ROS parameters
**Python CONSTANTS file** for application constants

### 13. Visualization
Standard RViz setup with custom markers for zones/recommendations

### 14. Documentation
**Code comments** + **Separate documentation files**

### 15. Git Workflow
**Commit frequently**: After every logical step
- NOT: Complete feature → one commit
- YES: Many small commits per feature

### 16. Team Context
Team project, but this phase is solo work

## Metis Review Findings

### Questions Answered (with defaults)

**1. Zone Detection Precision**
- **Default**: Odometry drift acceptable (10-20cm error margin)
- Zone transitions: Passive state tracking (no events)
- Boundary handling: Assign to nearest zone

**2. Obstacle Handling**
- **Option A**: Nav2 costmap handles it autonomously (dynamic avoidance)

**3. "Working on Real Robot" Definition**
- **Option A**: Code runs without errors, basic navigation works
- Perfect tuning deferred to Week 2
- Minimum: Robot responds to nav goals and builds map

**4. Gazebo World Fidelity**
- **Option A**: Approximate dimensions (4x4m box, rough obstacle placement)
- No textures or visual fidelity needed

**5. Launch File Structure**
- **Separate files**: `sim_bringup.launch.py` and `real_bringup.launch.py`
- Some duplication acceptable for clarity

### Key Directives from Metis

**Package Structure**:
- Single package: `nutrient_nexus_navigation`
- ament_python build system
- No premature package splitting

**Zone Detection**:
- Axis-aligned bounding boxes (NOT polygons)
- Format: `{min_x, max_x, min_y, max_y}`
- Publish on `/current_zone` topic (std_msgs/String)
- Outside all zones → publish "no_zone"

**Scope Guardrails** (MUST NOT):
- No sensor data processing (stub topics only)
- No decision logic
- No custom message types (use std_msgs)
- No map save/load
- No perfect Nav2 tuning for real robot
- No abstract factories, strategy patterns, or over-engineering

**QA Requirements**:
- All scenarios must be agent-executable
- No "user manually tests" acceptance criteria
- Evidence files for each scenario

### Edge Cases Documented
- Robot outside zones → publish "no_zone"
- SLAM fails → restart launch file (known limitation)
- Invalid zone YAML → log warning, default to no zones
- Overlapping zones → report first match (alphabetical)
- Odometry drift → accept for Week 1, AMCL deferred

## Research Findings

### Codebase State (from explore agent)
- **Clean slate**: No existing ROS2 workspace or packages
- No `src/`, `install/`, `build/`, `log/` directories
- No `package.xml`, `setup.py`, launch files, or config files
- Starting from scratch

### ROS2 Jazzy + TurtleBot3 Patterns (from librarian agent)

**Package Structure**:
- Use `ament_python` build type
- `setup.py` installs: package, launch files, resource marker
- `setup.cfg` routes executables to `lib/<package_name>`
- Launch files in `launch/` directory

**Nav2 + SLAM**:
- Use `slam_toolbox` for mapping (publishes `/map` and `map -> odom`)
- Nav2 consumes map without `nav2_amcl` during SLAM
- Launch pattern: `slam_toolbox` + `nav2_bringup navigation_launch.py`
- Critical: `use_sim_time:=true` for sim, `false` for real robot

**TurtleBot3 Burger**:
- Set `TURTLEBOT3_MODEL=burger` environment variable
- Simulation: `turtlebot3_gazebo empty_world.launch.py`
- Real robot: `turtlebot3_bringup robot.launch.py`
- Common topics: `/scan`, `/tf`, `/odom`, `/cmd_vel`, `/map`
- Common frames: `map`, `odom`, `base_link`

**Sim-to-Real Strategy**:
- Keep one codebase with shared params YAML
- Branch launch files: `sim_bringup.launch.py` vs `real_bringup.launch.py`
- Only swap hardware/simulator adapters, not navigation logic
- Critical: Align `use_sim_time` across all nodes

**RViz Markers**:
- Use `visualization_msgs/msg/Marker` and `MarkerArray`
- Set `header.frame_id` to `map` or `base_link`
- Common types: SPHERE, CUBE, TEXT_VIEW_FACING
- Must republish or set `lifetime` to keep visible

## Assumptions (validated)

- Single TurtleBot3 Burger robot
- Python-based ROS2 nodes
- Wooden box environment = ~2-3 crop zones with obstacles
- English language for logs/output
- Running on Linux with Wayland (based on flake.nix workaround)
- Nix flake environment already configured
