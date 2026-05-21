# Smart Waste Collection Robot - Digital Twin PoC

This project demonstrates a proof-of-concept for a smart waste collection system using a TurtleBot3 robot in a simulated Gazebo environment. The system uses the ROS 2 Navigation Stack (Nav2) to autonomously dispatch the robot to collect trash from bins based on their simulated fill levels.

## Prerequisites

Before you begin, ensure your system (e.g., Ubuntu 20.04) has the following installed:
* **ROS 2 Foxy Fitzroy:** [Official Installation Guide](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
* **Git:** `sudo apt install git`
* **colcon:** `sudo apt install python3-colcon-common-extensions`
* **Gazebo Simulator:** `sudo apt install gazebo11 ros-foxy-gazebo-ros-pkgs`
* **ROS 2 Navigation Stack (Nav2):** `sudo apt install ros-foxy-navigation2 ros-foxy-nav2-bringup`
* **TurtleBot3 Packages:** `sudo apt install ros-foxy-turtlebot3 ros-foxy-turtlebot3-simulations`

## Installation

Follow these steps to set up the project workspace.

1.  **Clone the Repository** 

    ```bash
    git clone https://github.com/anasnofal/projects.git
    ```

2.  **Navigate to Workspace Root**

    ```bash
    cd ~/projects
    ```

  

3.  **Build the Workspace**  
    It's good practice to start with a clean build.

    ```bash
    # Clean previous build artifacts
    rm -rf build/ install/ log/

    # Build the workspace using colcon
    colcon build --symlink-install
    ```
    *Note: `--symlink-install` is convenient as it allows you to change Python scripts without needing to rebuild every time.*

## Running the Simulation

Follow these steps in order. Each `ros2` command should be executed in a **new terminal**.

### Step 1: Source the Workspace in Every Terminal

Before running any command, you must source the workspace's setup file. This makes your custom packages available to ROS 2.

**Do this in every new terminal you open:**

```bash
cd ~/projects
source install/setup.bash
```

### Step 2: Launch the Environment and Navigation

This step starts the core simulation (Gazebo), the navigation stack (Nav2), and the visualization tool (RViz).

1. **Launch Gazebo World & Robot**
   In your **first terminal**, launch the Gazebo simulation. This should open the simulator window with your world and the TurtleBot3 model.

   ```bash
   # Ensure TURTLEBOT3_MODEL is set, e.g., export TURTLEBOT3_MODEL=burger
   ros2 launch my_custom_turtlebot3_gazebo empty_world.launch.py
   ```
2. ** Launch the Real Robot**
   In your **another terminal**, launch the real robot using these commands.
   ```bash
   # Ensure TURTLEBOT3_MODEL is set, e.g., export TURTLEBOT3_MODEL=burger
   ros2 launch turtlebot3_bringup robot.launch.py
   ```
    
3. **Launch the Navigation Stack (Nav2)**
   In a **new terminal**, launch Nav2 and provide it with your pre-made map.

   ```bash
   # This command uses the map from your 'my_turtlebot3_controller' package.
   # Adjust the path if necessary.
   ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$(ros2 pkg prefix my_turtlebot3_controller)/share/my_turtlebot3_controller/maps/big_map.yaml
   ```


### Step 3: Localize the Robot in RViz

This step is **critical** for Nav2 to know where the robot is.

1. Wait for Gazebo and RViz to fully load. The robot in RViz might appear at the wrong spot initially.
2. In the RViz toolbar, click the **"2D Pose Estimate"** button.
3. Look at the Gazebo window to see the robot's actual starting position and put the real robot in the same position and orientation.
4. In the RViz map, **click at that same location and drag an arrow** to match the robot's pose in Gazebo and real life .
5. You should see the robot's laser scan (dots) snap into alignment with the map walls. The robot is now localized.

### Step 4: Run The Application Nodes

If everything is working, open new terminals for each of your custom nodes. **Remember to `source install/setup.bash` in each one!**

* **Terminal A - Bin Sensor Node:**
  *(This node simulates the bins and their fill levels)*

  ```bash
  ros2 run my_turtlebot3_controller BinSensorMockNode
  ```

* **Terminal B - Navigation Executor Node:**
  *(This node receives navigation goals and commands Nav2)*

  ```bash
  ros2 run my_turtlebot3_controller navigation_executor_node
  ```

* **Terminal C - Decision Node:**
  *(This node contains the logic to decide which bin to visit)*

  ```bash
  ros2 run my_turtlebot3_controller DecisionNode
  ```

* **Terminal D - `cmd_vel` Relay Node :**
  *(Run this only if your Gazebo robot uses namespaced topics like `/my_tb3/cmd_vel`)*

  ```bash
  ros2 run my_turtlebot3_controller cmd_vel_relay_node
  ```

### Step 5: Expected Behavior

After starting all nodes and localizing the robot, the system should start operating autonomously.

1. The `BinSensorMockNode` terminal will show fill levels being published.
2. The `DecisionNode` terminal will eventually detect a full bin and log that it is "Dispatching robot...".
3. The robot in Gazebo and RViz should start moving towards the target bin.
4. Once it arrives, it will wait for 10 seconds, and you should see the `DecisionNode` command a reset, which is logged by the `BinSensorMockNode`.
5. The `DecisionNode` will then be ready to select the next full bin.

---

### Troubleshooting

* **Models (Trash Cans) Not Loading in Gazebo:**
  If your world opens but the trash can models are missing, Gazebo cannot find their model files. Ensure the model folders (e.g., `first_2015_trash_can`) are located in a path Gazebo can find. A common user location is the hidden `.gazebo` directory in your home folder: `~/.gazebo/models/`. You may need to copy your model folders there.

* **Map Not Loading in RViz:**
  If RViz is empty or shows map errors, check the terminal where you launched Nav2 (Step 2.2). Look for errors from `[map_server]`. This usually indicates a wrong path to the `map.yaml` file in your launch command.

* **Robot Freezes or Navigation Fails:**

    If the robot stops and the DecisionNode seems stuck waiting for a navigation result, the robot's localization may have drifted.

    1.In the Rviz , click the "Pause" button.
  
    2.In RViz, re-localize the robot using the "2D Pose Estimate" tool to give it a fresh, accurate position
  
    3.In the Rviz, click the "Resume" button. This may help Nav2 recover and continue its task.


