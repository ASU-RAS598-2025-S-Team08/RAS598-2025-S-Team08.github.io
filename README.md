# 🤖 Autonomous Warehouse Patrolling Robot  
### 🛠️ RAS 598 – Spring 2025 · Team 08  
Arizona State University · Prof. Daniel Aukes  

---

## 🔍 Overview

The **Autonomous Warehouse Patrolling Robot** is a ROS2-powered mobile robotic system developed on the TurtleBot4 platform to autonomously patrol warehouse environments. It leverages sensor fusion, SLAM/AMCL-based localization, path planning, and anomaly detection to provide real-time monitoring of indoor spaces.

This project demonstrates how low-cost hardware combined with modular, open-source software can address practical challenges in warehouse safety and automation.

---

## 🎯 Objectives

- Perform autonomous patrolling along designated routes
- Avoid static and dynamic obstacles using LiDAR, depth camera, and ultrasonic sensors
- Detect anomalies (unexpected objects or human presence) in real time
- Visualize live robot status and feedback through a Qt-based GUI
- Allow manual override and inspection via the GUI interface

---

## 🧠 Tech Stack

| Category         | Tools / Frameworks |
|------------------|--------------------|
| Robot Platform   | TurtleBot4         |
| Middleware       | ROS2 Humble        |
| Navigation       | Nav2 Stack (DWB Planner, SLAM / AMCL) |
| Sensors          | RPLiDAR, Intel RealSense, IMU, Ultrasonic |
| Fusion & Logic   | EKF via `robot_localization`, custom ROS2 nodes |
| Visualization    | RViz2, custom Qt GUI |
| Hosting & Docs   | GitHub Pages (MkDocs Material Theme) |

---

## 📂 Repository Structure

├── docs/ # Markdown docs for GitHub Pages
│ ├── index.md
│ ├── final_report.md
│ ├── code.md
│ ├── dataset.md
│ ├── videos.md
│ └── assets/ # Gantt chart, node graphs, etc.
├── .github/workflows/ # GitHub Actions for MkDocs deployment
│ └── deploy.yml
├── mkdocs.yml # MkDocs site configuration
└── README.md # This file


👉 **Live Project Website:**  
**🔗 https://asu-ras598-2025-s-team08.github.io/RAS598-2025-S-Team08.github.io/**

---

## 📦 Core ROS2 Packages

| Package             | Description                                     |
|---------------------|-------------------------------------------------|
| `patrol_manager`    | Patrol logic, route generation, replanning      |
| `anomaly_detector`  | Detects unexpected depth readings and objects   |
| `sensor_fusion_node`| Combines odometry, IMU, LiDAR via EKF           |
| `gui_backend`       | Sends logs, alerts, and control signals to GUI  |

---

## 🎥 Demo Videos

- 📦 **Autonomous Patrol Demo:**  
  [Watch on YouTube](https://youtu.be/e1mFo_xL-tc)

- 🎤 **Elevator Pitch:**  
  [Watch on YouTube](https://www.youtube.com/watch?v=srB0Ry8rDLg)

---

## 🧪 Dataset & Evaluation

📁 **Dataset (HuggingFace):**  


Includes:
- Raw sensor ROS bag files
- Annotated anomaly logs (CSV)
- Used to validate patrol coverage and detection accuracy

---

## 🧭 ROS2 Launch Instructions

To clone and launch:

```bash
cd ~/ros2_ws/src
git clone https://github.com/asu-ras598-2025-s-team08/warehouse-patrol-code
cd ..
colcon build
source install/setup.bash
ros2 launch patrol_manager main.launch.py
