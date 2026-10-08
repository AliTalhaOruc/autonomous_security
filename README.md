# 🛡️ Autonomous Military Reconnaissance & Threat Mapping System (Husky UGV)

[![ROS 2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Classic-orange.svg)](https://gazebosim.org/)
[![YOLOv8](https://img.shields.io/badge/YOLOv8-Ultralytics-00FFFF.svg)](https://docs.ultralytics.com/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)](LICENSE)

## Overview

This repository presents an autonomous reconnaissance, exploration, target detection, 3D localization, and threat mapping system developed for a **Husky UGV** in a custom military simulation environment.

The system combines **SLAM Toolbox, Nav2, custom frontier-based exploration, RGB-D perception, and a custom-trained YOLOv8 model**. Detected targets are localized in 3D using depth information and TF2 coordinate transformations, allowing their positions to be represented on the global map in **RViz**.

The project is being developed as a modular autonomous security and reconnaissance platform, with autonomous navigation and perception currently implemented and additional target-tracking and turret capabilities planned for future stages.

---

## Highlights

- **Custom Military Simulation Environment:** Gazebo Classic environment containing military structures, narrow passages, open areas, walls, vehicles, and target models.
- **Autonomous Exploration:** Custom frontier-based exploration system built on top of **SLAM Toolbox and Nav2** for autonomous map exploration without manual waypoint control.
- **Custom SLAM & Navigation Configuration:** Tuned SLAM Toolbox, Nav2 Hybrid-A* planning, Regulated Pure Pursuit control, costmaps, and robot motion parameters for the custom environment.
- **RGB-D Perception:** Simulated RGB-D camera integrated directly into the Husky platform for simultaneous visual and depth information.
- **Custom-Trained YOLOv8 Detector:** YOLOv8 model trained using data collected from the Gazebo simulation, including augmented samples and negative examples to reduce false detections.
- **2D-to-3D Target Localization:** Bounding-box pixel coordinates and depth information are converted into 3D camera coordinates and transformed into the global `map` frame using TF2.
- **Threat Verification:** Detection confidence and multi-frame confirmation logic are used to reduce transient false positives before registering targets on the map.
- **RViz Threat Visualization:** Confirmed targets can be represented as markers in the global map according to their threat classification.

---

## 🎬 Demo & Autonomous Patrol Simulation

### Autonomous Mapping & Threat Marker Placement

Demonstrates the Husky UGV autonomously navigating the military complex, detecting targets in real time, and populating color-coded threat markers on RViz using 3D depth fusion.


<video src="https://github.com/user-attachments/assets/0aa99d98-e885-41e5-a59d-4f037399e8e8" controls autoplay loop muted playsinline width="100%"></video>

---

## ✨ Main Features & Perception Pipeline

### 1. Autonomous Exploration, SLAM & Navigation

The Husky UGV autonomously explores unknown areas of the environment using a custom frontier-based exploration strategy.

The exploration system works together with:

- **SLAM Toolbox** for simultaneous localization and mapping
- **Nav2** for global planning and local control
- **Smac Hybrid-A\*** for path planning
- **Regulated Pure Pursuit** for path following
- Custom frontier scoring and goal selection
- TF2-based robot pose estimation

The navigation parameters were tuned specifically for the custom military environment, including narrow passages, open areas, obstacles, and large unexplored regions.

### 2. Custom Military World & Environment Design

A customized Gazebo Classic environment was created using multiple military and environmental models. The environment intentionally contains both narrow passages and large open areas to provide a challenging test environment for autonomous exploration and navigation.

The world includes structures, walls, vehicles, vegetation, military targets, and different types of obstacles to evaluate the robot under varied spatial and visual conditions.

<img width="1624" height="1101" alt="Screenshot from 2026-10-09 00-09-51" src="https://github.com/user-attachments/assets/e30ef12c-a398-411a-ab48-9b45d1cbc565" />

### 3. Custom YOLOv8 Threat Detector & Training

A custom YOLOv8 model was trained using images collected directly from the Gazebo simulation.

The training data contains different:

- Target distances and scales
- Viewing angles
- Simulation lighting conditions
- Target appearances
- Background structures and textures

Negative examples such as empty walls and visually similar simulation structures were also included to reduce false detections.

The detector currently supports target classes including:

- `tank`
- `soldier`
- `fire_station`
- `house`
- `stop_sign`

The detector processes the RGB camera stream and associates detections with depth information for subsequent 3D localization.

### 4. RGB-D 3D Spatial Localization

Instead of relying only on 2D image coordinates, the system uses the depth value corresponding to the detected target region.

The pipeline is:

```text
YOLO Bounding Box
        │
        ▼
Target Pixel Coordinates
        │
        ▼
Depth Measurement
        │
        ▼
Camera Intrinsic Parameters
        │
        ▼
3D Camera Coordinates
        │
        ▼
TF2 Transformation
        │
        ▼
Global Map Coordinates (X, Y, Z)
```

This allows a detected object to be associated with a physical position in the simulated environment rather than only being identified inside the camera image.

### 5. Smart Confirmation & False Positive Mitigation

Because simulated environments can contain visually similar textures, lighting artifacts, and structures, individual YOLO detections are not immediately treated as confirmed threats.

The detection system uses:

- Confidence thresholds
- Class-specific threat scores
- Multi-frame confirmation
- Spatial consistency checks
- Depth-based localization

A candidate target must satisfy the configured verification conditions before being registered as a confirmed threat on the global map.

This prevents short-lived visual misclassifications from unnecessarily generating threat markers.

---

## 📡 ROS 2 Interfaces

### Camera Input Topics

The RGB-D perception pipeline uses:

- `/camera/image_raw` — RGB image stream
- `/camera/depth/image_raw` — depth image
- `/camera/camera_info` — camera intrinsic parameters

### Navigation & Mapping Topics

The autonomous system works with ROS 2 interfaces including:

- `/scan` — LiDAR LaserScan
- `/map` — generated occupancy grid
- `/odom` — odometry
- `/odometry/filtered` — EKF-filtered odometry
- `/tf` and `/tf_static` — coordinate transformations

### Threat Visualization

Confirmed detections are published as visualization markers through:

- `/detected_threats` — `visualization_msgs/msg/MarkerArray`

These markers can be visualized directly in RViz on top of the generated global map.

---

## 🏗️ System Architecture

```text
                    ┌─────────────────────┐
                    │   Gazebo Military   │
                    │      World          │
                    └──────────┬──────────┘
                               │
                 ┌─────────────┴─────────────┐
                 │                           │
                 ▼                           ▼
          ┌─────────────┐             ┌─────────────┐
          │   LiDAR     │             │   RGB-D     │
          │   /scan     │             │   Camera    │
          └──────┬──────┘             └──────┬──────┘
                 │                           │
                 ▼                           ▼
          ┌─────────────┐             ┌─────────────┐
          │ SLAM Toolbox│             │   YOLOv8    │
          └──────┬──────┘             └──────┬──────┘
                 │                           │
                 ▼                           ▼
          ┌─────────────┐             ┌─────────────┐
          │ Autonomous  │             │ Depth +     │
          │ Exploration │             │ Camera Info │
          └──────┬──────┘             └──────┬──────┘
                 │                           │
                 ▼                           ▼
          ┌─────────────┐             ┌─────────────┐
          │    Nav2     │             │ 2D → 3D     │
          │ Navigation  │             │ Projection  │
          └──────┬──────┘             └──────┬──────┘
                 │                           │
                 │                           ▼
                 │                    ┌─────────────┐
                 │                    │    TF2      │
                 │                    │ Camera→Map  │
                 │                    └──────┬──────┘
                 │                           │
                 └──────────────┬────────────┘
                                ▼
                     ┌────────────────────┐
                     │ Global Map / RViz  │
                     │ Threat Markers     │
                     └────────────────────┘
```

---

## 📂 Repository Structure

```text
autonomous_security_ws
│
├── src
│   │
│   ├── autonomous_security
│   │   ├── autonomous_security
│   │   │   ├── explorer.py
│   │   │   └── yolo_detector.py
│   │   │
│   │   ├── config
│   │   │   ├── ekf.yaml
│   │   │   ├── husky_nav.yaml
│   │   │   ├── husky_slam_params.yaml
│   │   │   └── imu_filter.yaml
│   │   │
│   │   ├── launch
│   │   │   ├── bringup_simulation.launch.py
│   │   │   └── start_exploration.launch.py
│   │   │
│   │   ├── models
│   │   │   ├── best.pt
│   │   │   └── ...
│   │   │
│   │   └── world
│   │       └── mili.world
│   │
│   └── husky
│       ├── husky_description
│       ├── husky_control
│       ├── husky_gazebo
│       └── ...
│
├── .gitignore
└── README.md
```

---

## 🛠️ Installation & Setup

### Prerequisites

- **OS:** Ubuntu 22.04 LTS
- **ROS:** ROS 2 Humble
- **Simulator:** Gazebo Classic 11
- **Python:** 3.10+
- **GPU:** NVIDIA CUDA-capable GPU recommended for YOLO inference

### Build Instructions

```bash
# Clone the repository
git clone https://github.com/AliTalhaOruc/autonomous_security.git
cd autonomous_security

# Source ROS 2
source /opt/ros/humble/setup.bash

# Install ROS dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

---

## 🚀 Usage Guide

### 1. Launch the Simulation

```bash
ros2 launch husky_gazebo gazebo.launch.py
```

The custom military world is loaded automatically through the Husky Gazebo launch configuration.

### 2. Start Autonomous Exploration

```bash
ros2 launch autonomous_security start_exploration.launch.py
```

This starts the custom frontier-based exploration node together with the required navigation stack.

### 3. Start the YOLO Threat Detector

```bash
ros2 run autonomous_security yolo_detector
```

The detector receives the RGB-D camera data, performs YOLO inference, calculates target depth and 3D coordinates, and publishes confirmed threat markers.

---

## 💻 Development Environment

The system has been developed and tested using:

- Ubuntu 22.04 LTS
- ROS 2 Humble
- Gazebo Classic 11
- NVIDIA RTX 3060
- Python 3.10+
- OpenCV / CV Bridge
- Ultralytics YOLOv8
- SLAM Toolbox
- Nav2
- TF2

The perception and navigation modules are designed to operate together inside the same ROS 2 simulation environment.

---

## 🔮 Future Improvements

The following capabilities are planned for the next development stages:

- [ ] **Autonomous Gimbal Turret Integration:** Integrate the previously developed 2-axis autonomous gimbal turret with the Husky platform.
- [ ] **State Machine Integration:** Combine exploration, target detection, tracking, engagement, and mission completion into a unified state-machine architecture.
- [ ] **Kalman Filter Motion Prediction:** Extend the perception system with position and velocity estimation for moving ground and aerial targets.
- [ ] **Moving Target Tracking:** Track confirmed targets over time and update their predicted positions.
- [ ] **Autonomous Return:** Return the Husky to its starting position after completing the assigned reconnaissance mission.

---

## ✍️ Author

**Ali Talha Oruç**

Computer Engineering Student | Konya Technical University
