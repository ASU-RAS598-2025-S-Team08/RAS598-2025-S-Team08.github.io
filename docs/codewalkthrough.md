# Code Walkthrough

## ROS Packages

- `patrol_control`: Handles path planning and state transitions
- `sensor_fusion`: Combines data from LiDAR, IMU, Depth camera
- `anomaly_detection`: Runs anomaly classifiers and publishes events
- `gui_publisher`: Publishes real-time data for frontend display

## External Packages Used

| Package | Version | URL |
|--------|---------|-----|
| ROS2 Nav2 | Foxy | https://github.com/ros-planning/navigation2 |
| OpenCV | 4.5 | https://github.com/opencv/opencv |

## Installation & Usage

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 launch patrol_control patrol.launch.py
