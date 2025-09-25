<div align="center">
  <h1>QRB ROS Video Test Nodes</h1>
  <p align="center">
    <!-- Add images or videos to showcase your project demo, use case, or logo -->
  </p>
  <p>GStreamer-based test nodes for validating QRB ROS Video hardware acceleration capabilities</p>
  
  <a href="https://ubuntu.com/download/qualcomm-iot" target="_blank"><img src="https://img.shields.io/badge/Qualcomm%20Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white" alt="Qualcomm Ubuntu"></a>
  <a href="https://docs.ros.org/en/jazzy/" target="_blank"><img src="https://img.shields.io/badge/ROS%20Jazzy-1c428a?style=for-the-badge&logo=ros&logoColor=white" alt="Jazzy"></a>
  
</div>

---

## 👋 Overview

> 📌 **Introduction to the project**
> - GStreamer-based source and sink test nodes for video processing validation
> - Support for both CompressedImage and custom Image message types
> - Hardware acceleration testing capabilities for Qualcomm platforms
> - Comprehensive testing framework for video encoding/decoding workflows

> 📌 **Architecture diagrams**
> - Test nodes integrate with QRB ROS Video pipeline for end-to-end validation

> 📌 **Describe the architecture diagram:**
> - **Source Nodes**: Generate test video data from files and publish to ROS2 topics
> - **Sink Nodes**: Consume video data from ROS2 topics and write to output files
> - **GStreamer Integration**: Leverages GStreamer pipelines for media handling
> - **Message Types**: Support for standard CompressedImage and optimized custom Image formats
> - **Hardware Validation**: Tests hardware acceleration through the complete video pipeline

## 🔎 Table of Contents

  * [APIs](#-apis)
  * [Supported Targets](#-supported-targets)
  * [Installation](#-installation)
  * [Usage](#-usage)
  * [Build from Source](#-build-from-source)
  * [Contributing](#-contributing)
  * [FAQs](#-faqs)
  * [License](#-license)

## ⚓ APIs

### 🔹 Test Node ROS APIs

#### Source Nodes

**CompressedReader**
- **Published Topic**: `output`
- **Message Type**: `sensor_msgs/msg/CompressedImage`
- **Description**: Reads video files and publishes compressed image data
- **Supported Formats**: H.264, H.265, MP4

**ImageReader**
- **Published Topic**: `output`
- **Message Type**: `qrb_ros::transport::type::Image`
- **Description**: Reads raw video files and publishes uncompressed image data
- **Supported Formats**: Raw video (NV12)

#### Sink Nodes

**CompressedWriter**
- **Subscribed Topic**: `input`
- **Message Type**: `sensor_msgs/msg/CompressedImage`
- **Description**: Receives compressed image data and writes to files
- **Output Formats**: MP4, Raw compressed stream

**ImageWriter**
- **Subscribed Topic**: `input`
- **Message Type**: `qrb_ros::transport::type::Image`
- **Description**: Receives uncompressed image data and writes to files
- **Output Formats**: Raw video files

#### ROS Parameters

**Common Parameters (All Nodes):**
- `format` (string): Pipeline format - "raw" or "mp4"
- `width` (int): Frame width in pixels
- `height` (int): Frame height in pixels  
- `framerate` (string): Frame rate (e.g., "30/1")
- `log-level` (string): Logging level - "debug", "info", "warn"

**Source Node Parameters:**
- `url` (string): Path to input video file
- `duration` (string): Playback duration - "1d", "2h", "34m56s"

**Sink Node Parameters:**
- `url` (string): Path to output video file

### 🔹 GStreamer Pipeline APIs

#### Source Pipeline Templates

**H.264 Pipeline:**
```bash
filesrc ! h264parse ! stream-format=byte-stream,alignment=au ! appsink
```

**H.265 Pipeline:**
```bash
filesrc ! h265parse ! stream-format=byte-stream,alignment=au ! appsink
```

**MP4 Pipeline:**
```bash
filesrc ! qtdemux ! h264parse ! stream-format=byte-stream,alignment=au ! appsink
```

**Raw Video Pipeline:**
```bash
filesrc ! rawvideoparse ! appsink
```

#### Sink Pipeline Templates

**MP4 Output Pipeline:**
```bash
appsrc ! h264parse ! qtmux ! filesink
```

**Raw Output Pipeline:**
```bash
appsrc ! filesink
```

## 🎯 Supported Targets

- Qualcomm platforms with hardware video acceleration support
- Ubuntu systems with GStreamer and ROS2 Jazzy
- Systems with V4L2 video device support
- Hardware with DMA buffer capabilities

---

## ✨ Installation

> 📌 **For complete installation instructions, please refer to the main [QRB ROS Video Installation Guide](../../README.md#-installation)**

The test nodes are included with the main QRB ROS Video package installation. Once you have installed the main package following the instructions in the [main README.md](../../README.md), the test nodes will be available for use.

## 🚀 Usage
Launch the video encoder for real-time video compression:

**Example workflows:**

1. **Local Video File Recording:**
   ```bash
   # Preparation: 
   # push YUV sample file 1920_1080_nv12.yuv to /data/ directory

   # Execution:
   ros2 launch qrb_ros_video encoder_launch.py
   ```

2. **Local Video File Playback:**
   ```bash
   # Preparation: 
   # push YUV sample file 1920_1080.mp4 to /data/ directory

   # Execution:
   ros2 launch qrb_ros_video decoder_launch.py
   ```
---

## 👨‍💻 Build from Source

### Step 1: Install Dependencies

```bash
# Install build tools and dependencies
sudo add-apt-repository ppa:ubuntu-qcom-iot/qcom-ppa
sudo add-apt-repository ppa:ubuntu-qcom-iot/qirp
sudo apt update
sudo apt install build-essential cmake pkg-config

# Install GStreamer development packages
sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-devs

# Install ROS2 Jazzy (if not already installed)
# Follow instructions at https://docs.ros.org/en/jazzy/Installation.html
sudo apt install ros-jazzy-rclcpp ros-jazzy-rclcpp-components ros-jazzy-ament-cmake-auto ros-jazzy-std-msgs ros-jazzy-sensor-msgs ros-jazzy-qrb-ros-transport-image-type
```

### Step 2: Clone and Build

```bash
# Navigate to your ROS2 workspace
cd ~/ros2_ws/src

# Clone the repository (if not already cloned)
git clone https://github.com/qualcomm-qrb-ros/qrb_ros_video qrb_ros_video

# Build the test nodes
cd ~/ros2_ws
colcon build --packages-select qrb_ros_video --cmake-args -DBUILD_TESTING=ON

# Source the workspace
source install/setup.bash
```

## ❔ FAQs

<details>
<summary>What video formats are supported by the test nodes?</summary><br>
The test nodes support H.264, H.265, MP4 container format for compressed video, and raw NV12 format for uncompressed video. The specific format support depends on the GStreamer plugins available on your system.
</details>

<details>
<summary>How do I troubleshoot GStreamer pipeline issues?</summary><br>
Enable debug logging with `-p log-level:=debug` parameter. Also check that all required GStreamer plugins are installed and that input files exist and are accessible. Use `gst-inspect-1.0` command to verify plugin availability.
</details>

<details>
<summary>Can I use custom video files for testing?</summary><br>
Yes, you can use any video files that match the supported formats. Make sure the file path is accessible and the format matches the pipeline configuration (raw for uncompressed, appropriate codec for compressed files).
</details>

<details>
<summary>How do I verify that hardware acceleration is working?</summary><br>
Monitor system performance during video processing. Hardware acceleration should show significantly lower CPU usage compared to software-only processing. You can also check V4L2 device usage and monitor memory allocation patterns.
</details>

<details>
<summary>What are the performance expectations for the test nodes?</summary><br>
Performance depends on video resolution, frame rate, and hardware capabilities. The test nodes are optimized for real-time processing and should handle standard video formats (1080p@30fps) with minimal latency when hardware acceleration is available.
</details>