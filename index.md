# Project Name:  Autonomous Warehouse Goods Transportation System (Turtlebot Demonstration & Future Scope)
## Team Information
- **Team Number**: Team 08
- **Team Members**:
  - Ha Long Truong
  - Yashwanth Gowda
  
## Semester and Year:
- **Semester**: Spring 2025

## University, Class, Professor:
- **University**: Arizona State University
- **Class**: RAS598 - Experimentation and Deployment of Robotic Systems
- **Professor**: Daniel M. Aukes

## Project Plan: Autonomous Warehouse Goods Transportation System (Turtlebot Demonstration & Future Scope)

1. Project Overview and Research Question
The goal of this project is to develop a robotic system capable of autonomously transporting goods within a warehouse. The robot will be responsible for identifying products, navigating obstacles, and delivering items to specific storage locations. The key research question this project aims to answer is:

How can we design and implement an autonomous robotic system that efficiently and accurately transports goods in a dynamic, obstacle-rich warehouse environment?

This system will leverage advanced sensors (LiDAR, 3D cameras, barcode scanners) and real-time data integration to ensure seamless navigation and precise goods handling. The project will demonstrate its core capabilities through a small-scale demonstration using TurtleBots, with plans for scaling up to more advanced systems in the future.

![Project Figure: Could not be loaded](C:\Users\halon\OneDrive\Documents\ASU\RAS598\Team_Diagram.png)

2. Figure Communicating the Idea & Sensor Integration:
3. Sensor Integration
   
Sensors play a critical role in ensuring the robot’s ability to navigate and handle products correctly. During the demonstration and testing phases, the following sensors will be used:

LiDAR & 3D Cameras: These will be used for real-time navigation and obstacle avoidance. They provide spatial awareness of the robot’s environment, enabling it to detect and avoid obstacles in its path.

Barcode/QR Code Scanners: These sensors will be used for product identification, allowing the robot to recognize the goods being transported and confirm that the right item is taken to the correct location.

Weight/Size Sensors: These will be utilized to ensure that the goods are correctly sized and weighed before being placed in storage. This ensures that products are handled correctly and no errors occur during transportation.

For the small-scale demonstration, the TurtleBot will use its built-in LiDAR sensor for navigation and obstacle avoidance. Barcode scanning will be simulated using external devices integrated into the system. During testing, sensor data will be collected to validate the robot's navigation accuracy, obstacle avoidance capability, and proper handling of goods. In the final demonstration, real-time sensor data will allow the robot to adapt to dynamic changes in the environment, such as new obstacles or path adjustments.

Turtlebot Demonstration:

Code snippet

graph TD

    A[Goods Source] --> B(Barcode Scanner);
    B --> C{Product Identified?};
    C -- Yes --> D[Navigation (LiDAR)];
    C -- No --> E[Manual Intervention];
    D --> F[Simulated Delivery];
    F --> G[Webviz Visualization];
    E --> G;
    
Future Scope:

Code snippet

graph TD

    A[Goods Source] --> B(Barcode/QR Scanner & OCR);
    B --> C{Product Identified?};
    C -- Yes --> D(Weight/Size Sensors & 3D Point Cloud Analysis);
    C -- No --> E[Manual Intervention/Error Handling & AI Assisted Visual Inspection];
    D --> F{Verification Successful?};
    F -- Yes --> G[Path Planning & Navigation (LiDAR/3D Camera & Sensor Fusion)];
    F -- No --> E;
    G --> H[Autonomous Vehicle (AV) Movement & Motor Control];
    H --> I[Obstacle Avoidance (LiDAR/3D Camera & Predictive Algorithms)];
    I --> J[Storage Location & Precision Gripper Control];
    J --> K[Real-Time Data Visualization (Webviz/Node-RED) & AR Overlay];
    K --> L[Data Storage (MySQL) & Cloud Synchronization];
    L --> M[Warehouse Management System (WMS) & AI Demand Prediction];
    M --> N[Dynamic Storage Allocation & Route Optimization];
    N --> G;
    E --> K;

Sensor Integration:

Turtlebot Demonstration:

LiDAR data will be used for real-time mapping and obstacle avoidance.

A camera-based barcode scanner will identify products.

Sensor data will be processed using ROS 2 and visualized in Webviz.

During testing, sensor data will be logged to verify navigation accuracy.

The final demonstration will show real-time sensor data in Webviz.

Future Scope:

LiDAR and 3D camera data will be fused for accurate localization and obstacle detection.

Barcode/QR code scanners and OCR will be used for product identification.

Weight/size sensors and 3D point cloud analysis will verify product handling.

Sensor data will be used for real-time control, decision-making, and data visualization.

Testing will involve validating sensor accuracy, reliability, and fault tolerance.

The final demonstration will show the system's ability to adapt to dynamic environmental changes using sensor data.

3. Interaction:

To influence the behavior of the robot and allow for seamless interaction, we plan to develop a web-based interface. This interface will be used for:

Real-Time Monitoring: Users will be able to view the robot’s current status, including location, task progress, and any errors.

Control & Adjustment: Operators can adjust routes and reassign tasks as needed in real-time.

Data Visualization: Users can view sensor data in a graphical format to monitor robot performance, including navigation metrics and task completion times.

We will use Node-Red or Azure Twin to build this web-based interface. Node-Red will enable real-time data flow from the robot’s sensors, while Azure Twin will provide a more warehouse-centric visualization, including the robot's current location and task status. A mockup of this interface will feature a dashboard that includes a map of the warehouse, real-time sensor readings, and a log of goods being transported. This interface will allow easy monitoring and decision-making.

4. Control and Autonomy:

Turtlebot Demonstration:

The ROS 2 navigation stack will provide autonomous navigation.

LiDAR data will be used for obstacle avoidance.

A state machine will manage task execution.

The barcode scanner will trigger destination selection.

Future Scope:

Advanced control algorithms will be implemented for dynamic path planning and decision-making.

Sensor fusion will provide a robust representation of the environment.

AI-driven demand prediction will optimize storage allocation.

Multi-agent coordination will be implemented for multiple robots.

5. Preparation Needs:

To succeed in this project, several knowledge areas and skills must be covered:

Hardware Integration: We need to understand how to integrate various sensors with the robot hardware, such as connecting barcode scanners and weight sensors to the robot’s control system.

Sensor Data Processing: Knowledge of how to collect, filter, and refine sensor data for navigation and product handling is crucial.

Web-Based Interface Development: Skills in developing interactive dashboards for real-time monitoring and control of the robot will be necessary.

Control Systems and Autonomy: We will need a deep understanding of control algorithms and decision-making processes to create an autonomous robot that can navigate and perform tasks efficiently.

In class, topics related to ROS2, sensor integration, and web-based control systems will be crucial. Additionally, we may need specific guidance on using Node-Red or Azure Twin for the user interface.

6. Final Demonstration:

Turtlebot Demonstration:

The Turtlebot will autonomously navigate a simulated warehouse, scan barcodes, and deliver simulated products.

Resources: Turtlebot, LiDAR, camera, computer with ROS 2, simulated warehouse environment.

Classroom setup: A designated area for the simulated warehouse.

Environmental variability: The Turtlebot will handle minor variations in the environment.

Testing: We will track navigation accuracy and task completion.

Future Scope:

A full-scale demonstration will showcase the system's ability to handle complex tasks in a realistic warehouse.

Resources: Autonomous robots, advanced sensors, warehouse environment, web-based interface.

Classroom setup: A large space with simulated warehouse conditions.

Environmental variability: The system will adapt to dynamic changes using sensor data.

Testing: We will track transportation times, accuracy, and safety metrics.

7. Impact:

This project will have significant learning and practical implications:

It will deepen our understanding of autonomous robotic systems, particularly in warehouse automation.

We will gain experience in sensor integration, data processing, and control systems, which are essential skills in robotics and AI.

By developing the project from prototype to small-scale demonstration, we will learn how to scale and optimize robotic systems for real-world applications, making it applicable for future developments in industrial settings.

The work may also contribute to the development of new technologies in warehouse automation and autonomous logistics, providing valuable insights for further research.

8. Advising:
For this project, we will seek advising from Dr. Aukes (or another faculty member with expertise in automation). The advising will focus on:

ROS2 for robot control and sensor integration.

Guidance on Webviz, Node-Red, or Azure Twin for developing the web-based monitoring and control system.

Assistance with hardware integration and sensor selection (e.g., LiDAR, barcode scanners, weight sensors).

Dr. Aukes (or another advisor) will be expected to provide access to specialized hardware, advice on troubleshooting issues, and mentoring in developing the project’s autonomy and sensor integration. We will aim to meet with the advisor on a regular basis (e.g., weekly or bi-weekly) to track progress and refine our approach.

9. Weekly Milestones (Weeks 7-16):

Week	Hardware Integration	Interface Development	Sensors	Controls & Autonomy

7	Turtlebot setup	Webviz setup	LiDAR data acquisition	Basic navigation setup

8	Camera mounting	Node-RED installation	Barcode scanner integration	Sensor data integration

9	Simulated warehouse setup	Basic Dashboard creation	Sensor data filtering	Task execution state machine

10	Testing hardware setup	Refine dashboard design	LiDAR navigation testing	Refine Navigation control

11	Hardware testing and debugging	Begin Database planning	Barcode testing and Debugging	Task completion testing

12	Final turtlebot hardware adjustments	Database Implementation	System wide sensor testing	System wide control testing

13	Research advanced sensors	Begin Node-Red/MySQL integration	Research sensor fusion	Research advanced path planning

14	Planning full scale robot design	Design full scale dashboard	Planning system wide sensor implementation	Planning advanced control architecture

15	Planning full scale warehouse simulation	Database testing and implementation	Sensor fusion design	control architecture testing

16	Documentation and final testing	Finalize Dashboard and database	Final sensor testing	Final control testing
