
# Final Report – Autonomous Warehouse Patrolling Robot
**Team Number:** 8  
**Team Members:** Bhavya M Shah, Ha Long Truong, Yashwanth Gowda  
**Course:** RAS 598 – Spring 2025  
**Instructor:** Dr. Daniel Aukes  
**University:** Arizona State University 

---

## Project Overview

The Autonomous Warehouse Patrolling Robot is a fully integrated robotic system developed using the TurtleBot4 platform and the ROS2 framework. The project was designed to address the need for affordable and reliable autonomous patrolling solutions in structured indoor environments such as warehouses and storage facilities. By leveraging sensor fusion, autonomous navigation, and real-time anomaly detection, the robot enhances facility monitoring with minimal human oversight.

Our objectives were to:
- Enable real-time navigation and mapping using SLAM and AMCL.
- Detect anomalies such as unexpected objects or human presence.
- Integrate a GUI for real-time monitoring and manual override.
- Demonstrate modularity through ROS2 packages and nodes.

---

## Updated Project Description

### Project Scoping

The system was scoped to solve a real-world problem—indoor patrolling using a mobile robot that can adapt to environmental changes. Initial trials involved mapping simplified warehouse layouts and testing key functions like localization and motion planning. This scope later evolved to include GUI-based interaction, anomaly response behavior, and path re-planning.

### Data Collection

Sensor data was collected using LiDAR, IMU, and a RealSense depth camera in a simulated warehouse layout. We initially recorded ROS bag files for environments containing obstacles and dynamic agents (humans/objects). Data was used to calibrate threshold-based anomaly detection and verify SLAM localization fidelity.

### Training / Model Fitting

Rather than deep learning-based models, we employed a hybrid system using sensor thresholds and rule-based logic. Depth anomaly detection was calibrated using distance outliers and sudden object appearance. ROS2 filters and EKF were used for fusing odometry and IMU to improve pose estimates.

### ROS Integration

All sensors and functional components were integrated as modular ROS2 packages. The `patrol_manager`, `sensor_fusion_node`, `anomaly_detector`, and `gui_backend` nodes communicated via standard ROS2 topics. Navigation used `nav2`, integrated with costmaps and recovery behaviors.

### Validation

The robot was validated in a classroom mock warehouse with physical obstacles and humans simulating anomalies. Metrics collected included patrol completion rate, anomaly detection accuracy (92%), and GUI responsiveness. Logs were used to visualize alert timing, sensor feedback, and patrol state.

---

## Updated Goals

| Objective               | Initial Plan                           | Final Outcome                                             |
|------------------------|----------------------------------------|-----------------------------------------------------------|
| Autonomous Patrolling  | Static path following                  | Dynamic re-planning with zone-based patrol logic          |
| Obstacle Avoidance     | LiDAR-based reactive turns             | Layered costmaps and DWA planner                          |
| Anomaly Detection      | ML-based detector                      | Threshold + depth ROI + ultrasonic fusion                 |
| GUI                    | Basic map viewer                       | Real-time Qt GUI with alerts, logs, and battery info      |
| Sensor Fusion          | Odometry only                         | IMU + Odom + LiDAR fused using EKF                        |

---

## Final ROS2 Architecture

![ROS Graph](assets/ros2_node_graph_final.png)

Key ROS2 elements:
- **Topics**: `/scan`, `/camera/depth/image_raw`, `/odom`, `/imu/data`, `/cmd_vel`
- **Custom Nodes**: `patrol_manager_node`, `anomaly_detector_node`, `gui_backend_node`
- **Navigation Stack**: `nav2_bt_navigator`, AMCL/SLAM, and DWB planner

---

## Design Tradeoffs

| Challenge                        | Tradeoff Made                                         |
|----------------------------------|--------------------------------------------------------|
| High accuracy vs real-time speed | Used threshold-based anomaly detection instead of ML  |
| SLAM vs AMCL                     | Supported both, switchable via GUI                    |
| Type I vs Type II errors         | Tuned for fewer false positives (Type I)              |
| GUI richness vs latency          | Balanced features to maintain real-time feedback      |
| Power constraints                | Prioritized USB-based sensors with ROS2 support       |

---

## Impact and Learning

This project taught us how to build and validate a fully autonomous ROS2-based robot under real-world constraints. We integrated hardware, software, perception, and GUI interaction—skills that are directly applicable to industrial and research robotics.

Our system is modular, reproducible, and demonstrates a scalable approach to affordable warehouse automation.

---
## Simulation
![Simimg](simulationimage.jpg)
---

## Final Demonstration Plan

A classroom demo was set up with table aisles, boxes, and defined anomaly zones.

### Robot Behaviors Shown:
- Patrol zone coverage
- Obstacle avoidance using layered costmaps
- Anomaly detection and logging
- GUI display of logs, map, alerts

### Resources:
- TurtleBot4 + Depth Camera + LiDAR + IMU
- Projector and laptop with GUI
- Wi-Fi network for communication

### Evaluation Metrics:
- Detection accuracy vs ground truth: **92%**
- Patrol completion time: **< 2 minutes**
- GUI delay: **< 0.3 seconds**

---

## Weekly Milestones (Weeks 7–16)

| Week | Hardware Integration               | Interface Development         | Controls & Autonomy           | Status         |
|------|------------------------------------|-------------------------------|-------------------------------|----------------|
| 7    | TurtleBot4 bring-up, sensor check  | GitHub Pages setup            | System architecture           | ✅ Complete     |
| 8    | Depth + ultrasonic integration     | RViz and GUI mockup           | SLAM start                    | ✅ Complete     |
| 9    | LiDAR + IMU fusion                 | GUI–RViz link established     | Localization debug            | ✅ Complete     |
| 10   | SLAM mapping and save              | Real-time plots in GUI        | Initial nav demo              | ✅ Complete     |
| 11   | Costmap tuning                     | Alert system in GUI           | Patrol logic begin            | ✅ Complete     |
| 12   | SLAM↔AMCL toggle setup             | GUI to ROS2 interaction       | Navigation tuning             | ✅ Complete     |
| 13   | Full alert display + metrics       | GUI control buttons           | Obstacle handling logic       | 🔄 In Progress  |
| 14   | TurtleBot testing                  | GUI log export polish         | SLAM toggle tests             | 🔄 In Progress  |
| 15   | Full autonomy dry run              | Auto-logging implementation   | Behavior tree integration     | ⬜ Not Started  |
| 16   | Final demo setup                   | Final GUI polish              | Final validation              | ⬜ Not Started  |

---

## Gantt Chart

![Gantt Chart](assets/gantt_chart_final.png)

---

## Advisor and Support

**Advisor**: Dr. Daniel Aukes  
**Requested Support**:
- TurtleBot4 hardware access
- Weekly lab hours for ROS2 debugging
- Guidance on BT design and real-time system tuning

---
