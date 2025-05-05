
# 🚀 Launch Instructions – `warehouse_simulation-main`

This guide explains how to bring up the complete warehouse simulation and navigation stack from the `warehouse_simulation-main` project.

---

## 📦 Prerequisites

- ROS 2 Humble installed
- `ros2_ws` workspace created and built
- The `warehouse_simulation-main` directory placed in `~/ros2_ws/src/`
- Workspace built with `colcon build` and sourced

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## 🧱 Package Overview

| Package            | Description |
|--------------------|-------------|
| `custom_world_pkg` | Contains the warehouse Gazebo world and models |
| `path_planner_pkg` | Launches the Nav2 stack and sends goals |

---

## 🔧 Step-by-Step Launch

### 1. 🏭 Launch Gazebo Simulation

```bash
ros2 launch custom_world_pkg launch_simulation.launch.py
```

- Loads the TurtleBot4 into the custom warehouse Gazebo world
- Gazebo GUI will open

---

### 2. 🗺️ Launch Navigation Stack

Open a **new terminal**, and run:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch path_planner_pkg nav2.launch.py
```

This launches:
- Map server
- AMCL (localization)
- Behavior tree navigator
- Planner & controller servers

---

### 3. 🎯 Send Goal from Custom Node

Still in a separate terminal:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run path_planner mission_executor
```

This node:
- Sends a goal to `/navigate_to_pose`
- Uses the Nav2 BT Navigator to handle path planning & control

---

## 🎛 Visualize in RViz2

Optionally, launch `rviz2`:

```bash
rviz2
```

Configure to display:
- `/map`
- `/odom`
- `/path`
- `/tf`
- `/amcl_pose`
- `/goal_pose`

---

## 🧠 Final Flow Summary

```text
Gazebo ⟶ TF, scan, joint_states
 ↳ Nav2 Stack ⟶ planner, controller, AMCL
 ↳ mission_executor.py ⟶ sends custom goal
 ↳ RViz2 ⟶ real-time feedback and map visualization
```

---

## ✅ Output

Once launched:
- Robot will localize and navigate
- Paths will be visible in RViz2
- Simulation runs in Gazebo using your custom world

