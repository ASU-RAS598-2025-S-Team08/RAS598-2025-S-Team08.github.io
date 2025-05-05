
# 🧭 Full ROS 2 Architecture – `warehouse_simulation-main`

### 📦 Packages:
- `custom_world_pkg` → launches Gazebo with warehouse models
- `path_planner_pkg` → handles Nav2 stack and goal navigation

---

### 🔧 Nodes Identified

| Node Name              | Source                                 | Description |
|------------------------|----------------------------------------|-------------|
| `/gazebo`              | Gazebo Simulator                       | Physics & visualization engine |
| `/robot_state_publisher` | Auto-started in Gazebo                | Publishes joint TF from URDF |
| `/mission_executor`    | `mission_executor.py` in `path_planner_pkg` | Sends goals to `/navigate_to_pose` |
| `/map_server`          | Nav2 (launched via `planner.launch.py`) | Publishes static map |
| `/amcl`                | Nav2 localization                      | Computes robot pose on map |
| `/planner_server`      | Nav2 plugin                            | Plans global path |
| `/controller_server`   | Nav2 plugin                            | Executes local path |
| `/nav2_bt_navigator`   | Nav2 behavior tree                     | Orchestrates full navigation |
| `/rviz2`               | Visualization                          | GUI for maps, robot, goals |

---

### 🔌 Topics Flow

| Topic                            | From → To                         | Notes |
|----------------------------------|-----------------------------------|-------|
| `/navigate_to_pose/_action/goal`| `mission_executor` → `nav2_bt_navigator` | Sends goal coordinates |
| `/map`                           | `map_server` → `amcl`, `rviz2`    | Occupancy map for localization and display |
| `/scan`                          | `gazebo` (LiDAR plugin) → `amcl`  | Simulated 2D LiDAR |
| `/amcl_pose`                     | `amcl` → `nav2_bt_navigator`      | Robot pose estimate |
| `/plan_request`                 | `nav2_bt_navigator` → `planner_server` | Global path request |
| `/path`                          | `planner_server` → `controller_server` | Path handoff |
| `/cmd_vel`                       | `controller_server` → `gazebo`    | Robot wheel commands |
| `/tf`                            | `robot_state_publisher` → `rviz2` | Robot transforms |
| `/feedback`                      | `nav2_bt_navigator` → `rviz2`     | Goal feedback |
| `/joint_states`                  | `gazebo` → `robot_state_publisher`| Joint info for TF |

---

## 🔄 How the System Flows

```text
[mission_executor.py]
     |
     v
[navigate_to_pose] --[BT]--> [planner_server] --> [controller_server] --> [cmd_vel] --> [gazebo]

[amcl] <-- [scan] + [map] <-- [map_server]
    |
   [amcl_pose] --> [nav2_bt_navigator]

[tf], [map], [feedback] --> [rviz2]
```

---

## ✅ Diagram: Visual ROS2 Graph

📷 ![ROS2 Architecture Diagram](assets/ros2nodearch.png)

---

## 📌 Summary

The simulation launches:
- A robot inside a Gazebo warehouse
- Sends goals from a custom node (`mission_executor`)
- Uses the standard Nav2 stack to plan and navigate autonomously
- RViz2 for visualization and debugging
