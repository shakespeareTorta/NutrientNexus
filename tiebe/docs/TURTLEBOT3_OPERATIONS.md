# TurtleBot3 Operations Guide

NutrientNexus Week 1 — Running on the Physical Robot

---

## Prerequisites

**On the laptop (ROS2 Jazzy):**
- This repo built and sourced: `colcon build --packages-select nutrient_nexus_navigation && source install/setup.bash`
- Same ROS2 Jazzy installation as the robot
- Both devices on the same network

**On the TurtleBot3 Burger:**
- ROS2 Jazzy installed
- TurtleBot3 bringup packages installed
- `nutrient_nexus_navigation` package built and sourced (or accessible via the laptop's ROS environment)

---

## Network Setup

Both the laptop and TurtleBot3 must share the same ROS2 domain:

```bash
# Set on BOTH devices — must match
export ROS_DOMAIN_ID=30

# Verify they can see each other
ros2 topic list   # should show topics from both sides
```

If using a dedicated network interface, also set:
```bash
export ROS_LOCALHOST_ONLY=0
```

---

## Step 1: Start the Robot (on TurtleBot3)

SSH into the robot or use a direct terminal:

```bash
ssh ubuntu@<TURTLEBOT_IP>

# Source ROS2
source /opt/ros/jazzy/setup.bash

# If the package is installed on the robot:
source ~/workspace/install/setup.bash

# Start the robot hardware drivers
ros2 launch turtlebot3_bringup robot.launch.py
```

Leave this terminal running. The robot's LiDAR, motors, and odometry are now active.

---

## Step 2: Launch NutrientNexus on the Laptop

Open a new terminal on the laptop:

```bash
cd /path/to/CBL
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch nutrient_nexus_navigation real_bringup.launch.py
```

This starts:
- SLAM Toolbox (builds a map from LiDAR)
- Nav2 navigation stack
- Zone detector node (publishes `/current_zone`)

Expected output — no errors, and you should see:
```
[slam_toolbox]: Registering sensor...
[zone_detector]: Zone detector node started
```

---

## Step 3: Verify the System

In a new terminal on the laptop:

```bash
source install/setup.bash

# Check all nodes are running
ros2 node list
# Expected: /slam_toolbox, /bt_navigator, /controller_server, /zone_detector, etc.

# Check LiDAR is publishing
ros2 topic hz /scan
# Expected: ~5 Hz

# Check odometry
ros2 topic hz /odom
# Expected: ~30 Hz

# Check zone detection
ros2 topic echo /current_zone
# Expected: "no_zone" until robot moves into a defined zone
```

---

## Step 4: Open RViz (on the Laptop)

```bash
source install/setup.bash
rviz2 -d nutrient_nexus_navigation/rviz/nav.rviz
```

You should see:
- The robot model in the centre
- LiDAR scan points around the robot
- The map building in real time as the robot moves

If the map frame is not aligned, click **2D Pose Estimate** in RViz and click on the robot's approximate position on the map.

---

## Step 5: Test Basic Movement (Teleop)

In a new terminal:

```bash
source install/setup.bash
ros2 run turtlebot3_teleop teleop_keyboard
```

Controls:
- `w` — forward
- `s` — stop
- `a` / `d` — rotate left / right
- `x` — backward

Drive the robot around the wooden box environment. Watch the map build in RViz.

---

## Step 6: Check Zone Detection

The three zones are defined in `nutrient_nexus_navigation/config/zones.yaml`:

| Zone   | X range       | Y range       | Location (approx)     |
|--------|---------------|---------------|-----------------------|
| zone_a | -1.8 to -0.5  | -1.8 to -0.5  | Back-left corner      |
| zone_b |  0.5 to  1.8  |  0.5 to  1.8  | Front-right corner    |
| zone_c | -1.8 to -0.5  |  0.5 to  1.8  | Front-left corner     |

Drive the robot into each zone and watch the topic:

```bash
ros2 topic echo /current_zone
```

Expected output when inside zone_a:
```
data: zone_a
```

Expected output when outside all zones (e.g. near the centre obstacle):
```
data: no_zone
```

---

## Step 7: Autonomous Navigation

Once the map is built (takes 1-2 minutes of driving around):

1. In RViz, click **2D Nav Goal** (top toolbar)
2. Click and drag on the map to set a goal position and orientation
3. The robot will plan a path and navigate autonomously

Nav2 will avoid the centre obstacle automatically via the costmap.

**Note**: Navigation tuning for the real robot is deferred to Week 2. The robot should attempt navigation but may not be perfectly smooth.

---

## Troubleshooting

**Robot doesn't move after teleop command**
- Check `ROS_DOMAIN_ID` matches on both devices
- Verify `ros2 topic echo /cmd_vel` shows messages when you press keys
- Check the robot bringup terminal for errors

**`/scan` not publishing**
- The LiDAR may not have started — check the robot bringup terminal
- Try restarting `turtlebot3_bringup robot.launch.py` on the robot

**Map not building**
- SLAM needs LiDAR data — verify `/scan` is publishing at ~5 Hz
- Drive the robot slowly to give SLAM time to process

**`/current_zone` always shows `no_zone`**
- The zone coordinates assume the robot starts near the world origin (0, 0)
- If the robot's odometry starts at a different position, the zones will be offset
- For Week 1, this is acceptable — odometry drift is a known limitation

**Nav2 fails to reach goal**
- This is expected for Week 1 — tuning is deferred
- Try a closer goal or reduce `max_vel_x` in `config/nav2_params.yaml`

**`PackageNotFoundError` on launch**
- The package needs to be built and sourced: `colcon build && source install/setup.bash`
- The launch files have source-tree fallbacks but the installed path is preferred

---

## Evidence to Capture (for Task 12)

Save these to `.sisyphus/evidence/`:

```bash
# Terminal output from launch
ros2 launch nutrient_nexus_navigation real_bringup.launch.py 2>&1 | tee .sisyphus/evidence/task-12-real-bringup.txt

# Node list
ros2 node list > .sisyphus/evidence/task-12-node-list.txt

# Scan rate
ros2 topic hz /scan --window 10 2>&1 | head -20 > .sisyphus/evidence/task-12-scan-rate.txt
```

Also capture:
- Screenshot of RViz showing the map and robot
- Short video of the robot moving in the wooden box environment

---

## Shutting Down

```bash
# On the laptop — Ctrl+C in the launch terminal, then:
pkill -f "ros2 launch"

# On the robot — Ctrl+C in the bringup terminal
```
