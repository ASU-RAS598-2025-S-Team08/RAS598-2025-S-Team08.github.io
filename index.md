**Title or Home Page (index.md Update)**

- **Project Name:** Autonomous Warehouse Patrolling Robot
- **Team Number:** Team 8
- **Semester and Year:** Spring 2025
- **University, Class, Professor:** Arizona State University, RAS 598 Dr. Aukes

**Team Project Plan**

**High-Level Concept Introduction**

Our project centers around the design and implementation of an Autonomous Warehouse Patrolling Robot utilizing the TurtleBot4 platform, integrated with the Robot Operating System 2 (ROS2) framework. The overarching goal is to develop a scalable, low-cost robotic solution capable of performing real-time patrolling tasks in structured indoor environments such as warehouses or storage facilities.

**Research Question**

How can we leverage low-cost mobile robotics platforms to achieve reliable autonomous patrolling in structured indoor warehouse environments, with real-time anomaly detection and environmental adaptability?

This research question drives us to explore and implement a range of robotics technologies and concepts, particularly focusing on the integration of multiple sensor modalities (such as LiDAR, IMU, depth camera, and odometry), robust control algorithms, and autonomy pipelines that balance reactive behavior (e.g., obstacle avoidance) with deliberative planning (e.g., patrol route optimization).

By simulating realistic warehouse conditions, our project will investigate the following:

- Multi-Sensor Fusion: Combining data from various onboard sensors to generate a coherent model of the robot’s environment and enhance localization, mapping, and situational awareness.
- Autonomous Navigation & Patrolling: Implementing SLAM (Simultaneous Localization and Mapping) techniques alongside path planning algorithms to enable the robot to patrol pre-defined or dynamically generated routes.
- Real-Time Anomaly Detection: Using sensory input (such as depth and visual cues) to detect unexpected objects, humans, or hazards in the robot’s path or assigned patrol zones.
- Interactive System Monitoring: Developing a custom graphical user interface (GUI) to display live robot status, environment mapping, anomaly alerts, and control interfaces for manual override or remote supervision.
- Environmental Adaptability: Equipping the robot with the capability to adapt its behavior based on changing conditions such as blocked paths, dynamic obstacles, or signal loss, ensuring robustness in real-world applications.

Our approach aims not only to implement a working prototype of a patrolling robot but also to provide a generalized framework for deploying autonomous agents in structured environments. Ultimately, this project aspires to demonstrate how affordable hardware combined with modular software architecture can address real-world operational needs in industrial settings — with potential extensions into areas like inventory monitoring, safety inspection, and collaborative automation.
*Conceptual Figure**
*Conceptual Figure**
Figure 1: Conceptual Overview of Warehouse Patrolling Robot_

- TurtleBot4 with LiDAR, depth camera, IMU, and ultrasonic sensors.
- Navigation path mapped using SLAM.
- GUI mockup showing patrol progress, anomaly flags, and data logs.

**Sensor Integration**

Sensor integration lies at the core of our Autonomous Warehouse Patrolling Robot’s functionality, enabling perception, localization, navigation, and real-time decision-making. Our approach involves the strategic fusion of multiple sensor modalities, each contributing complementary data that enhances the robot’s understanding of its environment and its ability to operate reliably in dynamic warehouse settings.
- Acts as the primary sensor for **SLAM (Simultaneous Localization and Mapping)**.
    - Provides high-resolution 2D scans of the surroundings to detect **static and dynamic obstacles**, and helps generate occupancy grid maps.
    - LiDAR data will feed into both the **local costmap** for real-time obstacle avoidance and the **global planner** for long-term path navigation.
2. **Depth Camera:**
- Used for **object detection** and **human presence recognition**, particularly in areas where traditional LiDAR may not provide sufficient vertical detail.
    - Supports anomaly detection by flagging unexpected objects in patrol zones.
    - Enables **3D perception**, helpful for future extensions such as object classification or semantic mapping.
3. **IMU (Inertial Measurement Unit):**
    - Provides continuous feedback on the robot's **orientation and angular velocity**.
    - When fused with odometry and LiDAR data, improves **localization accuracy**, especially in motion-heavy scenarios or sensor dropouts.
    - Plays a key role in **dead-reckoning** when GPS is not available (as is common indoors).
4. **Ultrasonic Sensors:**
    - Placed strategically around the robot to provide **short-range obstacle detection**.
    - Useful for **redundant safety layers**, especially when navigating tight spaces or around unstructured, cluttered areas where other sensors might have blind spots.

**Integration in ROS2**

Each sensor will be managed through dedicated ROS2 nodes, ensuring modularity, scalability, and real-time communication via ROS topics. Integration strategy includes:

- **Sensor Nodes**: Each sensor runs in its own node or device-specific package (e.g., rplidar_ros2, realsense2_camera, micro_ros).
- **Sensor Fusion**: Using packages like **robot_localization**, we fuse IMU and odometry with LiDAR-based localization to generate a robust pose estimate (/odom → /map transform).
- **Filtering**: We apply techniques such as **moving average filters**, **EKFs (Extended Kalman Filters)**, and **range clipping** to clean and refine incoming data streams.
- **Visualization & Debugging**: During development, we utilize **RViz2** to visualize:
  - Real-time LiDAR scans and maps
  - Point clouds from the depth camera
  - IMU orientation data
  - Ultrasonic proximity markers
- **Localization**: We employ **AMCL (Adaptive Monte Carlo Localization)** for probabilistic localization based on the occupancy grid map and laser scans.

**Development & Demonstration Phases**

- **Development Phase**:
  - Raw sensor data will be visualized and logged for calibration and validation.
  - Sensor streams will be debugged individually and in fused configurations to ensure alignment and synchronization.
  - Thresholds and tuning parameters will be iteratively optimized for reliable performance.
- **Final Demonstration**:
  - The robot will autonomously patrol a defined warehouse-like indoor environment.
  - Real-time sensor data will inform **navigation**, **obstacle avoidance**, and **anomaly detection** logic.
  - The GUI will display:
    - Live occupancy map
    - Detected objects or humans
    - Sensor status and proximity alerts
  - The robot will respond dynamically to unexpected obstacles, showcasing robust multi-sensor awareness and safe patrolling behavior.

**Interaction Plan**

The system will include:

- A **custom ROS2-based GUI** (built using rqt and Python Qt) to display:
  - Robot live location on a warehouse map.
  - Obstacle detection and patrol path overlay.
  - Anomaly alerts (unauthorized personnel/object detection).
  - System logs (distance traveled, patrol time, power stats).

We will implement basic keyboard and voice control overrides for debugging and semi-autonomous intervention.
_Figure 2: Sample GUI Mockup_ 
**Control and Autonomy: Enabling Intelligent Warehouse Patrolling**
The autonomous capabilities of the warehouse patrolling robot are achieved through a carefully structured, layered control system. This hierarchical architecture allows for robust and adaptable operation, enabling the robot to navigate complex warehouse environments, react to unforeseen circumstances, and execute its patrolling mission effectively.

**1\. Low-Level Control: Precise Motion Execution**

At the foundational layer lies the low-level control system. This layer is responsible for the precise execution of motion commands sent by higher-level controllers. It directly interfaces with the robot's actuators, primarily the wheel motors.

- Wheel Velocity Commands: The primary function of this layer is to translate desired linear and angular velocities into individual wheel velocity commands. This involves kinematic models that account for the robot's geometry and wheel configuration.
- Odometry Feedback: To ensure accurate motion, the low-level controller relies on odometry feedback. Data from wheel encoders (or other internal sensors) is used to estimate the robot's current pose (position and orientation).
- Goal: The primary goal of the low-level control is to ensure smooth, accurate, and stable robot motion according to the commands received from the mid-level controller.

**2\. Mid-Level Control: Reactive Navigation and Safety**

The mid-level control system builds upon the capabilities of the low-level layer, focusing on the robot's interaction with its immediate surroundings and ensuring safe navigation.

- Obstacle Avoidance: This is a critical function of the mid-level control. By processing real-time data from the robot's suite of sensors (LiDAR, depth camera, ultrasonic sensors), this layer detects obstacles in the robot's path.
- Reactive Behaviors: Based on the sensor data, the mid-level controller implements reactive behaviors to avoid collisions. These behaviors might include slowing down, stopping, turning, or maneuvering around obstacles. Algorithms like Vector Field Histogram (VFH), Dynamic Window Approach (DWA), or similar reactive planning techniques are typically employed.

**3\. High-Level Autonomy: Intelligent Mission Execution**

The high-level autonomy system orchestrates the robot's overall mission, enabling it to perform complex tasks without continuous human intervention.

- Patrol Routing: This layer utilizes a pre-mapped representation of the warehouse, dividing it into logical zones or waypoints. Based on the desired patrol strategy (e.g., sequential zone coverage, prioritized areas), the high-level controller generates a sequence of target locations or paths for the robot to follow.
- Behavior Trees: Behavior trees provide a powerful and modular framework for defining the robot's high-level behaviors and decision-making processes. They allow for the representation of complex sequences, conditional actions, and parallel tasks involved in the patrolling mission (e.g., navigate to zone A, scan for anomalies, log data, proceed to zone B).
- ROS2 Navigation Stack Integration: The robot leverages the ROS2 Navigation stack, a robust and widely used framework for mobile robot navigation. This stack provides pre-built functionalities for path planning, obstacle avoidance, localization, and map management.
- Autonomous Decision-Making: The high-level system enables the robot to make autonomous decisions regarding patrol routes based on factors like time, priority, or previously detected anomalies.
- Dynamic Path Re-planning: If the mid-level control encounters an unforeseen obstacle that blocks the planned path, the high-level autonomy system can trigger path re-planning. Utilizing the current map and the robot's current location, the Navigation stack can compute a new, feasible path to reach the original goal or the next patrol waypoint.
- Anomaly Detection Integration: This layer will also integrate with the robot's perception system to process anomaly detection results. Upon detecting an anomaly, the high-level controller can deviate from the standard patrol route to investigate, log the event, and potentially trigger alerts.

In summary, the layered control and autonomy system allows the warehouse patrolling robot to:

- Execute precise movements based on high-level commands (low-level control).
- Navigate safely and reactively to dynamic environments by avoiding obstacles (mid-level control).
- Intelligently plan and execute patrol missions based on pre-defined strategies and adapt to changing circumstances (high-level autonomy).
- Leverage the power of ROS2 Navigation for robust path planning and navigation capabilities.

This sophisticated control architecture ensures that the robot can effectively and efficiently perform its warehouse patrolling tasks, enhancing security, safety, and operational awareness within the facility.

**Preparation Needs**

To succeed, we need a deeper understanding of:

- Multi-sensor fusion in ROS2
- Navigation stack configuration and tuning
- Behavior trees and FSMs in autonomous systems
- GUI development using rqt and Python Qt

**Class Topics Requested:**

- Detailed coverage of ROS2 Navigation stack
- Real-time system debugging techniques
- Sensor data filtering and fusion strategies

**Final Demonstration Plan**

Our final demonstration is designed to showcase the full capabilities of the Autonomous Warehouse Patrolling Robot, developed using the TurtleBot4 platform and ROS2. The demo will simulate a realistic indoor warehouse environment using a scaled mockup constructed within a classroom. This hands-on trial will allow observers to evaluate the robot’s ability to autonomously patrol, avoid obstacles, detect anomalies, and present live system data through a custom-built graphical user interface (GUI).

**🧪 Demo Description**

During the demonstration, **TurtleBot4** will autonomously navigate through a **mock warehouse layout** configured using tables, cardboard boxes, and tape-marked pathways. It will:

- **Perform a patrol loop** based on predefined waypoints.
- Use **sensor data** from LiDAR, depth camera, IMU, and ultrasonic sensors to perceive and react to the environment.
- **Avoid obstacles** (both static and dynamic) in real-time.
- Provide **live telemetry and visualization** on a projector-based GUI that displays:
  - Real-time map (from RViz)
  - Robot position and path
  - Obstacle alerts
  - Anomaly flags (e.g., unexpected objects or human detection)

The demo will highlight key features such as localization accuracy, real-time responsiveness, safety, and user interaction.

**🛠️ Resources Needed**

To conduct the final demonstration, we will require the following components:

- **TurtleBot4** equipped with LiDAR, depth camera, IMU, and onboard compute.
- A pre-generated **warehouse map** loaded into RViz for AMCL-based localization.
- **Mock warehouse setup**: Classroom space arranged with:
  - Tables and **cardboard boxes** to simulate shelving.
  - Clear lanes representing robot patrol paths.
- **Wi-Fi router** for enabling ROS2 multi-node communication over the network.
- A **projector and monitor** setup to display the custom GUI for real-time interaction and visualization.

**🏗️ Classroom Setup**

- We will create aisles between tables and boxes to simulate warehouse lanes.
- "Anomaly zones" will be marked with boxes or human presence for the robot to detect and log.
- The TurtleBot4 will follow a set patrol route defined in the map and updated via the navigation stack.
- A central station (laptop or workstation) will run the GUI and visualization tools, connected to the robot over Wi-Fi.

**🌍 Environmental Adaptability**

To ensure robustness and adaptability in a non-static environment:

- **AMCL (Adaptive Monte Carlo Localization)** will handle localization uncertainty by continuously updating the robot’s pose using LiDAR scans and the known map.
- **Dynamic costmaps** in the navigation stack will enable the robot to avoid moving obstacles (e.g., people or displaced objects) and re-route in real time.
- Sensor data fusion ensures redundancy and resilience if one sensor becomes noisy or fails temporarily.

**✅ Testing & Evaluation Plan**

To verify system readiness and ensure a successful demo, we have implemented a multi-tiered testing framework:

1. **Unit Testing**:
    - Validate functionality of individual ROS2 nodes, including sensor input processing, map generation, and GUI backend.
2. **Functional Testing**:
    - Test integration between navigation, sensor fusion, and GUI components.
    - Validate that anomaly detection correctly flags unexpected objects or people.
3. **Live Evaluation Metrics**:
    - **Coverage Assessment**: Ensure the robot completes a full patrol loop and visits all waypoints.
    - **Anomaly Detection Accuracy**: Compare flagged anomalies to actual placements.
    - **Obstacle Avoidance Response Time**: Measure latency in responding to new obstacles.
    - **GUI Usability**: Confirm that the GUI provides clear, real-time, actionable feedback for users.

Video recording and real-time logging will be used to analyze performance and validate mission success.

**Impact**

This project pushes us to explore real-world robotic deployments using ROS2, a critical skill for modern robotic systems. It encourages learning across:

- Sensor integration
- Autonomous control
- System design
- GUI development It could influence future coursework by offering a modular demo project template. It will also prepare us for robotics research or industrial roles in automation and logistics.

**Advising**

- **Advisor:** Dr. Aukes
- **Requested Resources:**
  - Access to TurtleBot4
  - Weekly project check-ins
  - Use of robotics lab for after-hours testing
- **Advisor Expectations:**
  - Demonstrable progress every 2 weeks
  - Functional final demo
  - Full integration of perception, planning, and interaction modules

**Weekly Milestones (Gantt/Table Format)**

| **Week** | **Hardware Integration** | **Interface Development** | **Sensors** | **Controls & Autonomy** |
| --- | --- | --- | --- | --- |
| 7   | Initial TurtleBot4 setup | GUI layout skeleton | Verify LiDAR and camera streams | Basic teleoperation setup |
| 8   | Connect all sensors | GUI live sensor stream | Depth camera calibration | Low-level motor control |
| 9   | Odometry testing | Map view integration | IMU testing and filtering | PID tuning |
| 10  | SLAM with LiDAR | Add patrol status display | Sensor fusion trial | Path planning trial |
| 11  | Warehouse map creation | Implement alert logging | Dynamic obstacle detection | Obstacle avoidance tuning |
| 12  | Integrate patrol behavior | Add user override | Human/object detection using depth | FSM or Behavior Tree setup |
| 13  | System testing in lab | GUI final integration | Real-world condition testing | Re-planning and fallback logic |
| 14  | Demo dry-run #1 | Final visual polish | Final filtering refinements | Robustness testing |
| 15  | Debugging and patching | Optimize performance | Sensor noise reduction | Emergency stop tuning |
| 16  | Final demonstration prep | Presentation visuals | Run-time logs and metrics export | Final behavior integration |
