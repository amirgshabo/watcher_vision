# ZED SDK Depth Testing with VSO Files

This project is designed for testing depth functionality using VSO (Visual SLAM Object) files from StereoLabs with the ZED SDK.

## Project Structure

```
depth-vso-testing/
├── src/                 # Source code files
├── data/               # VSO files and test data
├── output/             # Generated depth maps and results
├── build/              # Build artifacts
├── CMakeLists.txt      # CMake configuration
├── requirements.txt    # Python dependencies (if using Python)
└── README.md          # This file
```

## Prerequisites

1. **ZED SDK**: Install the latest ZED SDK from StereoLabs
   - Download from: https://www.stereolabs.com/developers/release/
   - Follow installation instructions for your platform

2. **CMake**: Required for building C++ applications
   ```bash
   sudo apt install cmake
   ```

3. **OpenCV**: For image processing (optional but recommended)
   ```bash
   sudo apt install libopencv-dev
   ```

## VSO Files

VSO (Visual SLAM Object) files contain recorded ZED camera data including:
- Stereo images
- IMU data
- Depth information
- Camera calibration

Place your `.vso` files in the `data/` directory.

## Building the Project

```bash
cd depth-vso-testing
mkdir -p build
cd build
cmake ..
make
```

## Usage

### 🎯 **Two Approaches Available:**

#### **1. Standalone Applications (Recommended for VSO Analysis)**
```bash
# Simple processing (StereoLabs pattern)
python src/simple_vso_processor.py --input data/your_file.vso --frames 100

# Advanced analysis with ROS2-style processing  
python src/depth_analyzer.py --input data/your_file.vso --output output/

# C++ Application
./build/depth_tester data/your_file.vso --output output/
```

#### **2. ROS2 Nodes (For Real-time & Distributed Processing)**
```bash
# Quick ROS2 setup
./run_ros2_pipeline.sh

# Manual ROS2 nodes
ros2 run zed_depth_ros2 zed_depth_publisher.py --ros-args -p vso_file:=data/your_file.vso
ros2 run zed_depth_ros2 zed_depth_subscriber.py  # In another terminal
```

### **When to Use Each:**

| Use Case | Recommended Approach | Why |
|----------|---------------------|-----|
| **VSO file analysis** | Standalone Scripts | Direct file access, simpler setup |
| **Live camera processing** | ROS2 Nodes | Real-time streaming, topic-based |
| **Research/prototyping** | Python standalone | Faster iteration |
| **Production deployment** | C++ or ROS2 | Better performance |
| **Robot integration** | ROS2 Nodes | Standard robotics pipeline |

## Output

The application will generate:
- Depth maps (PNG/EXR format)
- Point clouds (PLY format)
- Analysis results (CSV/JSON format)

## Notes

- Ensure your VSO files are compatible with your ZED SDK version
- For best results, use VSO files recorded at appropriate resolution and frame rate
- Check ZED SDK documentation for supported VSO file formats and versions