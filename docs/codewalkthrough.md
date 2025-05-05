
# Code Walkthrough – Autonomous Warehouse Patrolling Robot

This walkthrough describes the structure and functionality of our ROS2 codebase, which powers the robot's simulation, navigation, and goal-based patrolling inside a custom warehouse environment.

---

## Packages Overview

| Package              | Purpose                                          |
|----------------------|--------------------------------------------------|
| `custom_world_pkg`   | Contains the Gazebo world, models, and launch files for the simulated warehouse environment |
| `path_planner_pkg`   | Implements ROS2 navigation, custom goal dispatching, and maps for SLAM & AMCL |

---

## `custom_world_pkg`

### ▶ Purpose

This package defines a **custom Gazebo warehouse world** complete with shelves and patrol lanes. It allows realistic simulation of patrolling behavior in structured environments.

### Key Structure

```
custom_world_pkg/
├── launch/
│   └── launch_simulation.launch.py      # Launches Gazebo with custom world
├── models/                              # Custom shelves, obstacles, etc.
├── worlds/
│   └── warehouse.world                  # Main world file used in simulation
├── package.xml / CMakeLists.txt         # Standard ROS2 package setup
```

### Launch the Simulation

```bash
ros2 launch custom_world_pkg launch_simulation.launch.py
```

This opens Gazebo with a preloaded warehouse layout and the TurtleBot4 robot.

---

## `path_planner_pkg`

### ▶ Purpose

This is the control and navigation package. It provides:
- Map loading and localization
- Navigation using Nav2 stack
- Code to send custom navigation goals

### Key Structure

```
path_planner_pkg/
├── launch/
│   └── nav2.launch.py                   # Launches map server, AMCL, Nav2 stack
├── maps/
│   ├── map.yaml                         # Metadata for static map
│   └── map.pgm                          # Grayscale occupancy map
├── path_planner/
│   └── custom_goal_sender.py           # Python node to send goals programmatically
├── setup.py / setup.cfg                # Python package setup
├── resource/ / test/                   # ROS2-specific folders
```

### Core File: `custom_goal_sender.py`

This node:
- Initializes a ROS2 node using `rclpy`
- Publishes a `PoseStamped` goal to the Nav2 stack
- Logs success/failure from the `NavigateToPose` action client

#### Example Usage:

```bash
ros2 run path_planner custom_goal_sender
```

Inside the script, you can define a goal pose like:

```python
goal_pose.pose.position.x = 2.0
goal_pose.pose.position.y = 3.5
```

You can modify this to send patrol waypoints dynamically, or integrate it with a behavior tree.

---

## How the Packages Work Together

```text
[custom_world_pkg] → Launches Gazebo simulation with TurtleBot4
        |
        v
[path_planner_pkg] → Loads map, enables SLAM/AMCL, sends goals to Nav2
        |
        v
[Nav2 Stack] → Handles path planning and obstacle avoidance
```

---

## Testing & Debugging

- Use `RViz2` with `/map`, `/tf`, and `/goal_pose` to visualize movement
- Run `ros2 topic echo /amcl_pose` to verify localization
- Adjust robot starting pose and goals in the launch file for better realism

---

## 🗂️ Repository Access

You can find the complete codebase here:  
📦 [warehouse_simulation-main](https://github.com/ASU-RAS598-2025-S-Team08/warehouse_simulation)
