
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
git clone https://github.com/ASU-RAS598-2025-S-Team08/warehouse_simulation.git
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

## 🚀 How to Use the Webapp

1. Clone and build the workspace:

```bash
mkdir ~/webapp/
cd ~/webapp/
git clone https://github.com/ASU-RAS598-2025-S-Team08/Webapp.git
```

2. Stream data (We used FastDDS to get the data):

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

3. Run backend:

```bash
cd ~/webapp/backend/
node server.js
```

4. Run frontend:

```bash
cd ~/webapp/frontend/
npm run dev
```

---

## 🎥 Video Demonstration

- Simulation Demo: Watch Here : [Watch our demo video](https://youtu.be/e1mFo_xL-tc?si=nqHWN2RykVblXiYx)

 
---

## 👥 Team Members

- Bhavya M Shah
- Ha Long Truong
- Yashwanth Gowda

---

## 📚 License

 Arizona State University | RAS598 – Dr. Daniel Aukes
