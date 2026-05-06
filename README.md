# ARCS WATCHER – Object Detection System (Vision Subsystem)

Real-time YOLO-based object detection for the ARCS WATCHER project.

This repository contains the Vision subsystem for WATCHER.  
It runs YOLOv8 object detection on ZED camera frames (live or from SVO recordings) and publishes detection messages (bounding boxes, class IDs, confidence scores) to ROS 2 topics used by the Depth, Tracking, and Mapping subsystems.

---

## Overview

**Pipeline:**  
ZED Camera / SVO Files → YOLO Detection → `/yolo_detections` → Downstream Modules

**Frameworks & Tools:**
- ROS 2 Jazzy  
- YOLOv8 (Ultralytics)
- Docker (CPU-based for development)  
- OpenCV (for SVO playback)

**Primary Output:**  
- `/yolo_detections` → `vision_msgs/Detection2DArray`  
- `/yolo_annotated` → Annotated images with bounding boxes  
- `/yolo_detections_with_depth` → Detections fused with ZED depth data  
- Bounding boxes, class IDs, confidence scores  

---

## System Architecture

### Input  
- **Live:** `/zed/zed_node/rgb/image_rect_color` – ZED2 / ZED2i RGB frames  
- **Recorded:** `/camera/rgb/image` – SVO file playback  

### Output  
`/yolo_detections` (Detection2DArray), consumed by:  
- **Depth Sensing** → Adds 3D coordinates  
- **Tracking** → Tracks objects across frames  
- **Mapping** → Builds global spatial map  

---

## Data Flow Diagram

~~~~
      ┌────────────────┐
      │   ZED Camera   │
      │  or SVO File   │
      └───────┬────────┘
              │  /zed/zed_node/rgb/image_rect_color
              │  /zed/zed_node/depth/depth_registered
              │
      ┌───────▼──────────────────────┐
      │      YOLO Detector Node       │
      │   (YOLOv8 + ROS2 Jazzy)       │
      │   confidence threshold: 0.45  │
      │   target: person, car,        │
      │   traffic light, stop sign    │
      └───────┬──────────────────────┘
              │  publishes:
              │  /yolo_detections
              │  /yolo_detections_with_depth
              │  /yolo_annotated
              │
   ┌──────────▼─────────────┐
   │   Depth Sensing Node    │
   │ (Detections + depth →   │
   │      3D positions)      │
   └──────────┬──────────────┘
              │  publishes:
              │  /object_positions_3d
              │
   ┌──────────▼─────────────┐
   │     Tracking Node       │
   │  (Track objects across  │
   │         frames)         │
   └──────────┬──────────────┘
              │  publishes:
              │  /object_tracks
              │
   ┌──────────▼─────────────┐
   │     Mapping Node        │
   │   (Global environment   │
   │         map)            │
   └─────────────────────────┘
~~~~

---

## What's New — Spring 2026

- **Live ZED Camera Integration** — detector node now subscribes directly to `/zed/zed_node/rgb/image_rect_color` for real-time detection
- **Depth Fusion** — bounding boxes are matched with ZED depth values to estimate object distance in meters
- **Tunable Config** — confidence threshold, NMS, target classes, and depth range configurable via `config/detector_params.yaml`
- **Class Filtering** — detection limited to relevant classes: person, car, traffic light, stop sign
- **ROS 2 Launch File** — single command to launch the full vision pipeline with the ZED camera
- **Performance Logging** — latency and detection count logged every 30 frames

---

## Quick Start — Live ZED Camera

### Requirements
- Ubuntu 24.04 with ROS 2 Jazzy
- ZED SDK installed
- ZED2 or ZED2i camera connected via USB 3.0

### Step 1: Clone Repository
~~~~bash
git clone https://github.com/amirgshabo/watcher_vision.git
cd watcher_vision
~~~~

### Step 2: Build the Package
~~~~bash
cd ros2_jazzy_ws
colcon build --packages-select arcs_yolo_detector
source install/setup.bash
~~~~

### Step 3: Launch with ZED Camera

*Terminal 1 — Start ZED Camera:*
~~~~bash
ros2 launch zed_wrapper zed2.launch.py
~~~~

*Terminal 2 — Start YOLO Detector:*
~~~~bash
ros2 launch arcs_yolo_detector detector_launch.py
~~~~

*Terminal 3 — Verify Output:*
~~~~bash
ros2 topic echo /yolo_detections
ros2 topic echo /yolo_detections_with_depth
~~~~

---

## Quick Start — Recorded Data (Rosbag)

**No ZED camera or Jetson needed!** Use our pre-recorded rosbag dataset.

### Step 1: Clone Repository
~~~~bash
git clone https://github.com/amirgshabo/watcher_vision.git
cd watcher_vision
~~~~

### Step 2: Download Rosbag Dataset

**Download Link:** [Street Detections Dataset (Google Drive)](https://drive.google.com/file/d/19ERBubKotWDy918HaFgf6tWvkFrFF3Xq/view?usp=sharing)

**Dataset includes:**
- 26 seconds of HD street scenes (1920x1080)
- 221 camera images
- 102 YOLO detections (cars, pedestrians, traffic)
- 116 annotated visualizations
- File size: ~2 GB (compressed)

### Step 3: Extract Rosbag
~~~~bash
unzip street_detections.zip
mv street_detections ros2_jazzy_ws/rosbags/
~~~~

### Step 4: Play the Rosbag
~~~~bash
ros2 bag play ros2_jazzy_ws/rosbags/street_detections --loop
~~~~

### Step 5: Launch Detector in Rosbag Mode
~~~~bash
ros2 launch arcs_yolo_detector detector_launch.py use_sim_time:=true input_topic:=/camera/rgb/image
~~~~

### Step 6: Subscribe to Topics
~~~~bash
ros2 topic echo /yolo_detections
ros2 topic echo /camera/rgb/image
ros2 topic echo /yolo_annotated
~~~~

---

## Configuration

All parameters tunable via `ros2_jazzy_ws/src/arcs_yolo_detector/config/detector_params.yaml`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `model_path` | yolov8n.pt | YOLO model weights file |
| `confidence_threshold` | 0.45 | Minimum detection confidence |
| `nms_threshold` | 0.50 | Non-maximum suppression threshold |
| `input_topic` | /zed/zed_node/rgb/image_rect_color | RGB image input topic |
| `depth_topic` | /zed/zed_node/depth/depth_registered | ZED depth topic |
| `target_classes` | [0, 2, 9, 11] | COCO class IDs to detect |
| `max_depth_range` | 10.0 | Max depth range in meters |

---

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/rgb/image` | sensor_msgs/Image | Raw RGB camera frames |
| `/yolo_detections` | vision_msgs/Detection2DArray | Object detections (bbox, class, score) |
| `/yolo_detections_with_depth` | vision_msgs/Detection2DArray | Detections with depth in meters |
| `/yolo_annotated` | sensor_msgs/Image | Annotated images with bounding boxes |

---

## Requirements

### For Using Recorded Data (All Teams)
- Ubuntu 24.04 with ROS 2 Jazzy
- ~4 GB disk space for rosbag
- No special hardware needed!

### For Live Development (Vision Team)
- Docker Desktop (Windows/Mac/Linux)
- ZED2 or ZED2i camera (for live capture)
- SVO files (for recorded playback)

---

## Vision Team Development Guide

### A. Practice at Home (Recommended Starting Point)

#### Windows Setup

**Prerequisites:**
- Docker Desktop with WSL 2
- Git for Windows
- PowerShell

**Step 1: Clone Repository**
~~~~bash
git clone https://github.com/amirgshabo/watcher_vision.git
cd watcher_vision
git config core.autocrlf false
git rm --cached -r .
git reset --hard
~~~~

**Step 2: Build Docker Image** (One-time, ~10 minutes)
~~~~bash
cd docker
docker build -f jazzy-dev.Dockerfile -t arcs_jazzy_ready .
cd ..\ros2_jazzy_ws
~~~~

**Step 3: Start Container**
~~~~bash
docker run -it --rm -v "${PWD}:/ros2_ws" --name arcs_dev arcs_jazzy_ready bash
~~~~

**Step 4: Setup Environment** (Inside Container)
~~~~bash
sed -i 's/\r$//' /ros2_ws/start.sh
chmod +x /ros2_ws/start.sh
/ros2_ws/start.sh

pip3 install ultralytics --break-system-packages
pip3 install "numpy<2" --break-system-packages

colcon build --packages-select arcs_yolo_test arcs_yolo_detector
source install/setup.bash
~~~~

**Step 5: Run the Pipeline**

*Terminal 1 - SVO Publisher:*
~~~~bash
ros2 run arcs_yolo_test svo_publisher_opencv --ros-args -p svo_file:=/ros2_ws/test_data/your_file.svo
~~~~

*Terminal 2 - YOLO Detector:*
~~~~bash
docker exec -it arcs_dev bash
source /ros2_ws/install/setup.bash
ros2 run arcs_yolo_detector yolo_detector --ros-args -p input_topic:=/camera/rgb/image
~~~~

*Terminal 3 - Verify Output:*
~~~~bash
docker exec -it arcs_dev bash
source /ros2_ws/install/setup.bash
ros2 topic echo /yolo_detections
~~~~

**Step 6: Record Rosbag for Team**
~~~~bash
ros2 bag record \
  /camera/rgb/image \
  /yolo_detections \
  /yolo_annotated \
  -o /ros2_ws/rosbags/my_recording
~~~~

#### Linux / Mac Setup
~~~~bash
docker run -it --rm -v $(pwd):/ros2_ws --name arcs_dev arcs_jazzy_ready bash

/ros2_ws/start.sh
pip3 install ultralytics --break-system-packages
pip3 install "numpy<2" --break-system-packages
colcon build --packages-select arcs_yolo_test arcs_yolo_detector
source install/setup.bash
~~~~

---

### B. Full System (Jetson + Live ZED Camera)

**Hardware Setup:**
- NVIDIA Jetson with ZED SDK installed
- ZED2 or ZED2i camera connected via USB 3.0

*Terminal 1 - ZED Camera:*
~~~~bash
ros2 launch zed_wrapper zed2.launch.py
~~~~

*Terminal 2 - YOLO Detector:*
~~~~bash
ros2 launch arcs_yolo_detector detector_launch.py
~~~~

*Terminal 3 - Monitor:*
~~~~bash
ros2 topic echo /yolo_detections
~~~~

---

## Troubleshooting

**"numpy version conflict" or segmentation fault:**
~~~~bash
pip3 uninstall numpy -y --break-system-packages
pip3 install "numpy<2" --break-system-packages
~~~~

**"bash: /ros2_ws/start.sh: bad interpreter"**
~~~~bash
sed -i 's/\r$//' /ros2_ws/start.sh
chmod +x /ros2_ws/start.sh
~~~~

**"docker: command not found"**
- Ensure Docker Desktop is running

**YOLO detector crashes after ~270 frames:**
- Known numpy compatibility issue
- Record rosbags quickly (15-30 seconds) before crash
- Or restart the detector and continue

**Empty rosbag (0 messages):**
- Ensure both SVO publisher and YOLO detector are running before recording
- Check topics exist: `ros2 topic list`

---

## Repository Structure

~~~~
watcher_vision/
├── docker/
│   └── jazzy-dev.Dockerfile
├── ros2_jazzy_ws/
│   ├── src/
│   │   ├── arcs_yolo_detector/
│   │   │   ├── arcs_yolo_detector/
│   │   │   │   ├── yolo_detector_node.py
│   │   │   │   └── __init__.py
│   │   │   ├── config/
│   │   │   │   └── detector_params.yaml
│   │   │   ├── launch/
│   │   │   │   └── detector_launch.py
│   │   │   ├── package.xml
│   │   │   └── setup.py
│   │   └── arcs_yolo_test/
│   ├── rosbags/
│   └── start.sh
├── .gitignore
└── README.md
~~~~

---

## Contributing

**Team Workflow:**
1. Create feature branch: `git checkout -b feature/your-feature`
2. Make changes and test locally
3. Record demo rosbag if applicable
4. Commit: `git commit -m "Description"`
5. Push: `git push origin feature/your-feature`
6. Create Pull Request on GitHub

---

## Project Information

**Team:** ARCS WATCHER  
**Project:** Wheelchair Assist Technology and Co-bot Helper Robot  
**Organization:** CSUN ARCS  
**Vision Team Lead:** Amir Shabo  

**Repository:** https://github.com/amirgshabo/watcher_vision  
**Dataset:** [Download from Google Drive](https://drive.google.com/file/d/19ERBubKotWDy918HaFgf6tWvkFrFF3Xq/view?usp=sharing)

---

## License

This project is part of the ARCS WATCHER senior design project at CSUN.
