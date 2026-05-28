# NutrientNexus 🌾🤖

**NutrientNexus** is an autonomous, sustainable agricultural robotics system powered by ROS 2 and Gazebo. Designed as a proof-of-concept Digital Twin, it demonstrates how precision agriculture can be automated to optimize crop yields while strictly adhering to sustainability goals—specifically mitigating agricultural runoff to protect marine ecosystems (SDG 14: Life Below Water).

---

## 🎯 System Overview

The system simulates a TurtleBot3 navigating a custom L-shaped agricultural arena containing four distinct crop zones and a base station. The robot autonomously patrols these zones, acts as a mobile sensor platform, and makes context-aware decisions to apply water (irrigate) or nutrients (fertilize). 

Crucially, NutrientNexus is a **Sustainability-First** system:
- It evaluates real-time simulated weather conditions and soil vulnerability.
- It will **halt** and refuse to fertilize a zone if there is a high risk of nutrient runoff (e.g., heavy rain incoming on highly vulnerable soil).
- All critical sustainability interventions are recorded on an immutable JSON ledger for auditing.

---

## 🏗️ Architecture & Nodes

NutrientNexus is built on a modular ROS 2 architecture, utilizing the Nav2 (Navigation 2) stack for autonomous movement and SLAM Toolbox for mapping. 

The application logic is divided into the following custom nodes:

### 1. The Brain: `CropDecisionNode`
The core intelligence of the robot. It operates a robust Finite State Machine (FSM):
*   **IDLE**: Waits for resources.
*   **NAVIGATING**: Dispatches the robot to the next priority crop zone.
*   **VERIFYING_ZONE**: Uses TF2 transforms to physically confirm the robot has entered the correct bounding box in the map.
*   **SCANNING**: Simulates taking physical soil samples over a few seconds.
*   **DECIDING**: The core logic engine. It reads moisture, nutrients, weather, and vulnerability scores. It then decides whether to Irrigate, Fertilize, or Halt (due to runoff risk).
*   **ACTUATING**: If treatment is safe and needed, the robot physically spins (dynamic duration/speed based on deficit and risk) and publishes treatment commands.
*   **COOLDOWN**: A brief pause before moving to the next zone.
*   **RETURNING_TO_BASE**: Triggered when battery (<30%) or fertilizer (<20%) runs low.

### 2. Environmental Simulation: `field_sensor_mock_node`
Acts as the digital twin of the agricultural field. It manages the internal state of all 4 zones, slowly drying them out and depleting nutrients over time. It dynamically shifts weather patterns and replenishes zone stats when it receives treatment commands from the robot.

### 3. Resource Management: `robot_resource_node`
Simulates the robot's physical limitations. It calculates the actual distance driven by the robot (via `/odom`) to drain the battery, and depletes the fertilizer tank whenever a spray command is executed. It refills both when the robot returns to the base station.

### 4. Navigation Interface: `navigation_executor_node`
A clean bridge between the `CropDecisionNode` and the complex Nav2 Action Server (`/navigate_to_pose`). It handles dispatching goals, tracking progress, and reporting success/failure back to the brain.

### 5. Localization: `zone_detector_node`
Listens to `tf2` transforms (specifically `map` -> `base_footprint`) to calculate the robot's exact physical coordinates and determines which logical zone bounding box the robot is currently occupying.

### 6. Safety & Auditing: `safety_stop_node` & `sustainability_audit_node`
*   **Safety Stop**: A defense-in-depth node that directly intercepts velocity commands (`/cmd_vel_nav`) and halts the robot if the LiDAR (`/scan`) detects an imminent collision.
*   **Audit**: Listens for critical blocked actions (e.g., "Refused to fertilize Zone 3 due to High Runoff Risk") and permanently logs them to an audit ledger file.

### 7. Visualization: `dashboard_node`
A lightweight Flask web server running in a background thread that serves a live, auto-updating HTML dashboard. It displays robot telemetry, zone health, and recent audit logs.

---

## 🗺️ The Arena & Zones

The system uses a custom Gazebo world (`new_world.world`) featuring an L-shaped room. The base station is located in the center of the room `(X: 0.8, Y: -1.4)`, and the field is divided into four perfectly tiled quadrants:

*   **Zone 0 (Green)**: Upper Left `(-0.4, 0.0)`
*   **Zone 1 (Blue)**: Upper Right `(1.8, 0.0)`
*   **Zone 2 (Yellow)**: Lower Right `(1.8, -2.6)`
*   **Zone 3 (Red)**: Lower Left `(-0.4, -2.6)`

---

## 🚀 How to Build and Run

### Prerequisites
*   Ubuntu 24.04
*   ROS 2 Jazzy
*   Gazebo Sim (Harmonic)

### 1. Build the Workspace
Open a terminal in the root of the workspace (where the `src` folder is) and build the packages:
```bash
colcon build --symlink-install
```

### 2. Launch the System
Source the workspace and launch the master bringup file. This single command starts Gazebo, RViz, Nav2, the SLAM Toolbox, the ROS-GZ bridge, and all 7 custom NutrientNexus nodes!

```bash
source install/setup.bash
ros2 launch my_turtlebot3_controller nexus.launch.py
```

### 3. Observe the Magic
1.  **RViz**: Watch the robot map the room using SLAM and navigate to its first waypoint.
2.  **Terminal**: Read the rich log output from the `CropDecisionNode` as it scans, decides, and actuates.
3.  **Dashboard**: Open a web browser and navigate to `http://localhost:5000` to view the live NutrientNexus telemetry dashboard.

---

## 🐳 Docker Support

To run the simulation in an isolated environment, a `Dockerfile` is provided.

### Build the Docker Image
From the root of the repository (where the `Dockerfile` is located):
```bash
docker build -t nutrient_nexus .
```

### Run the Docker Container
*Note: Running GUI applications (Gazebo/RViz) from Docker requires X11 forwarding, which varies by host OS. The below command is for Linux hosts.*

```bash
xhost +local:root
docker run -it --rm \
    --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    nutrient_nexus
```
*Inside the container, you can execute the build and launch commands listed above.*
