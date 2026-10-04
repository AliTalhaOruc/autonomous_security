# 🛡️ Autonomous Military Reconnaissance & Threat Mapping System (Husky UGV)

[![ROS 2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Classic-orange.svg)](https://gazebosim.org/)
[![YOLOv8](https://img.shields.io/badge/YOLOv8-Ultralytics-00FFFF.svg)](https://docs.ultralytics.com/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)](LICENSE)

## Overview
This repository presents a fully autonomous reconnaissance, target detection, 3D localization, and threat mapping system deployed on a **Husky UGV** inside a complex military simulation environment.

The system combines **SLAM/Autonomous Mapping** with a multi-sensor perception pipeline (RGB-D Camera + Custom YOLOv8) to detect military threats (tanks, soldiers, etc.), project their 2D detections into **3D coordinate space via TF2 transformations**, and dynamically register threat markers on **RViz** with high-precision confidence verification filters.

---

## Highlights
- **Custom Military Simulation Environment:** Complex Gazebo world featuring narrow corridors, structures, and wide open areas.
- **RGB-D Sensor Fusion & 3D Spatial Localization:** Real-time transformation of pixel targets ($c_x, c_y$) and depth values into real-world map coordinates ($X, Y, Z$).
- **Custom-Trained YOLOv8 Detector:** Trained on military targets with extensive data augmentation and negative sampling to reduce false positives.
- **Smart Confirmation Buffer:** Multi-frame confirmation counter (`CONFIRMATION_THRESHOLD = 5`) and class-specific confidence filtering to eliminate transient misclassifications.
- **Dynamic RViz Threat Mapping:** Color-coded threat classification markers updated dynamically on the global map.
- **Optimized Video Pipeline:** Streamlined OBS remuxing and FFmpeg processing for high-fidelity evaluation recording without simulation frame drops.

---

## 🎬 Demo & Autonomous Patrol Simulation

### Autonomous Mapping & Threat Marker Placement
Demonstrates the Husky UGV autonomously navigating the military complex, detecting targets in real time, and populating color-coded threat markers on RViz using 3D depth fusion.



<video src="src/autonomous_security/autonomous_security/media/demo_patrol_4x.mp4" 
       controls 
       autoplay 
       loop 
       muted 
       playsinline 
       width="100%">
</video>

---

## ✨ Main Features & Perception Pipeline

### 1. Custom Military World & Environment Design
Designed a customized, complex Gazebo Classic environment incorporating custom military meshes, dense urban corridors, and open observation points to stress-test SLAM mapping and object recognition logic under challenging visual occlusions.

<img width="1331" height="1175" alt="Screenshot from 2026-10-03 14-24-59" src="https://github.com/user-attachments/assets/f8b84166-7b40-45f7-b462-7b9760d1383b" />

### 2. Custom YOLOv8 Threat Detector & Training
- **Custom Dataset:** Images collected directly from Gazebo simulation runs featuring various scales, dynamic lighting conditions, angles, and distances.
- **Robustness Measures:** Integrated negative sampling (e.g., empty walls, simulation textures) and heavy augmentation to minimize false alarms in complex structural environments.
- **Output:** Publishes bounding boxes, class labels, depth-aligned target centers, and confidence scores over dedicated ROS 2 topics.

### 3. 3D Spatial Localization Pipeline (2D-to-3D Point Cloud Projection)
Instead of relying strictly on 2D visual frames, the system extracts the precise depth value at the target center pixel from the aligned RGB-D depth frame. Utilizing **TF2 coordinate frame transformations** (`camera_color_optical_frame` $\rightarrow$ `map`), the target's exact $X, Y, Z$ spatial vector in the global map frame is computed in real time.

### 4. Smart Confirmation Buffer & False Positive Mitigation
To prevent transient detection glitches (e.g., a wall temporarily misclassified as a threat due to visual reflection) from corrupting the mission map:
- **Class-Specific Thresholding:** Custom minimum confidence thresholds per target type (e.g., `tank: 0.85`, `soldier: 0.40`).
- **Confirmation Counter:** A candidate threat must persist in the same spatial vicinity across 5 consecutive frames (`CONFIRMATION_THRESHOLD = 5`) before being permanently locked and published as a global RViz marker.

---

## 📡 ROS 2 Interfaces

### Published Topics
- `/threat_markers` (`visualization_msgs/msg/MarkerArray`): Dynamic RViz markers representing detected threats.
- `/yolo/detections` (`vision_msgs/msg/Detection2DArray`): Processed YOLO bounding box and classification outputs.
- `/threat_localization/pose` (`geometry_msgs/msg/PoseStamped`): 3D map coordinates of confirmed targets.

### Subscribed Topics
- `/camera/color/image_raw` (`sensor_msgs/msg/Image`): RGB visual stream from camera.
- `/camera/depth/image_raw` (`sensor_msgs/msg/Image`): Aligned depth map.
- `/tf` & `/tf_static` (`tf2_msgs/msg/TFMessage`): Coordinate frame transformations.

---

## 🏗️ Perception & Localization Pipeline Architecture
Gazebo RGB-D Camera
              │
    ┌─────────┴─────────┐
    ▼                   ▼

[RGB Frame]        [Depth Frame]
│                   │
▼                   │
┌───────────────┐           │
│ YOLOv8 Engine │           │
└───────┬───────┘           │
│ Bounding Box      │
▼                   ▼
┌───────────────────────────────┐
│     2D-to-3D Projection       │
│  (Pixel Depth & Intrinsic Matrix) │
└───────────────┬───────────────┘
│ Camera Frame (x,y,z)
▼
┌───────────────────────────────┐
│    TF2 Transformation Node    │
│  (camera_optical ──► map)     │
└───────────────┬───────────────┘
│ Global Map Coordinates (X,Y)
▼
┌───────────────────────────────┐
│ Smart Confirmation Buffer     │
│ (Thresholding & 5-Frame Lock) │
└───────────────┬───────────────┘
│ Confirmed Threat
▼
┌───────────────────────────────┐
│   RViz Marker Publisher Node  │
└───────────────────────────────┘

---

## 📂 Repository Structure
autonomous_security_ws
│
├── src
│   ├── husky_security_description
│   │   ├── config/              # Nav2 & SLAM parameter configurations
│   │   ├── launch/              # Simulation & system launch files
│   │   ├── urdf/                # Xacro / URDF robot models & sensor attachments
│   │   └── worlds/              # Custom military simulation world
│   │
│   └── husky_threat_detector
│       ├── weights/             # Custom trained YOLOv8 model weights (best.pt)
│       ├── yolo_detector.py     # YOLOv8 detection & RGB-D fusion node
│       ├── threat_localizer.py  # TF2 transformation & confirmation buffer node
│       └── marker_publisher.py  # RViz dynamic threat marker generator
│
├── .gitignore
├── requirements.txt             # Python dependencies
└── README.md

---

## 🛠️ Installation & Setup

### Prerequisites
- **OS:** Ubuntu 22.04 LTS
- **ROS Version:** ROS 2 Humble
- **Simulator:** Gazebo Classic 11
- **Python:** 3.10+
- **GPU Acceleration:** NVIDIA CUDA-enabled GPU (RTX 3060 or higher recommended)

### Build Instructions

# Clone the repository
git clone [https://github.com/AliTalhaOruc/autonomous-security-ugv.git](https://github.com/AliTalhaOruc/autonomous-security-ugv.git)
cd autonomous_security_ws

# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y
pip install -r requirements.txt

# Build workspace
colcon build --symlink-install
source install/setup.bash

🚀 Usage Guide
Bash

# 1. Launch Gazebo Military World & Husky UGV
ros2 launch husky_security_description simulation.launch.py

# 2. Launch SLAM / Autonomous Navigation
ros2 launch husky_security_description navigation.launch.py

# 3. Launch Perception & Threat Mapping Pipeline
ros2 run husky_threat_detector yolo_detector
ros2 run husky_threat_detector threat_localizer

💻 Performance & System Specs

Tested on: Ubuntu 22.04 LTS | ROS 2 Humble | NVIDIA RTX 3060

    Inference Speed: Real-time GPU-accelerated YOLOv8 detection at 30+ FPS.

    Mapping Accuracy: Low-drift 3D point cloud transformation under dynamic map exploration.

    Recording Optimization: Handled via OBS remuxing and FFmpeg post-processing to avoid CPU simulation throttling during full autonomous runs.

🔮 Future Improvements (Roadmap)

    [ ] Autonomous Gimbal Turret Integration: Mounting the 2-axis autonomous pan-tilt turret on top of the Husky UGV.

    [ ] State Machine Integration: Hierarchical state machine to unify patrolling, dynamic threat engagement, firing control, and return-to-launch (RTL).

    [ ] Kalman Filter Motion Prediction: Extending tracking algorithms to intercept high-speed moving ground/aerial targets.

✍️️ Author

Ali Talha Oruç

Computer Engineering Student | Konya Technical University
