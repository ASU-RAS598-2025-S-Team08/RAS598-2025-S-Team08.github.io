
# 🤖 Autonomous Warehouse Patrolling Robot

A ROS 2-based simulation project built on TurtleBot4 for indoor autonomous navigation, anomaly detection, and warehouse patrolling using Gazebo and the Nav2 stack.

---

## 📦 Project Overview

This project demonstrates a scalable and low-cost robotic solution for structured warehouse environments. It uses multi-sensor fusion (LiDAR, IMU, Depth) and layered ROS 2 control architecture to enable fully autonomous patrols.

### 🔍 Features
- Simulated warehouse world in Gazebo
- Autonomous navigation using Nav2 + AMCL
- Custom goal sender via mission_executor node
- Real-time anomaly detection and alerts
- GUI integration with live RViz feedback
- Evaluation metrics and visualization tools

---

## 🧠 Architecture Diagram

![ROS2 Architecture](docs/assets/ros2nodearch.png)

---

## 🚀 How to Run

1. Clone and build the workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/YOUR_USERNAME/warehouse_simulation-main.git
cd ..
colcon build
source install/setup.bash
```

2. Launch simulation:

```bash
ros2 launch custom_world_pkg launch_simulation.launch.py
```

3. Launch Nav2 and goal logic:

```bash
ros2 launch path_planner_pkg nav2.launch.py
ros2 run path_planner mission_executor
```

4. (Optional) Visualize in RViz:

```bash
rviz2
```

---

## 📊 Evaluation Snapshots

- ![Navigation Accuracy](docs/assets/navigation_accuracy_graph.png)
- ![Anomaly Detection](docs/assets/anomaly_detection_graph.png)
- ![Working Flowchart](docs/assets/working_flowchart_robot.png)
- ![Gantt Chart](docs/assets/final_gantt_chart.png)

---

## 🎥 Video Demonstration

- Elevator Pitch: [Watch here](https://www.youtube.com/embed/YOUR_PITCH_VIDEO_ID)
- Simulation Demo: [Watch here](https://www.youtube.com/embed/YOUR_DEMO_VIDEO_ID)

---

## 👥 Team Members

- Bhavya M Shah
- Ha Long Truong
- Yashwanth Gowda

---

## 📂 Repository Structure

```
├── custom_world_pkg/          # Gazebo world and models
├── path_planner_pkg/          # Nav2 launch + mission_executor
├── docs/                      # MkDocs site content + assets
├── index.md                   # Full team report
└── README.md                  # This file
```

---

## 📚 License

MIT License – 2025 | Arizona State University | RAS598 – Dr. Daniel Aukes
