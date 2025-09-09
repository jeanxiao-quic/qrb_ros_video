<div align="center">
  <h1>QRB ROS Video</h1>
  <p align="center">
    <!-- Add images or videos to showcase your project demo, use case, or logo -->
  </p>
  <p>Hardware-accelerated video processing package for Qualcomm robotics platforms</p>
  
  <a href="https://ubuntu.com/download/qualcomm-iot" target="_blank"><img src="https://img.shields.io/badge/Qualcomm%20Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white" alt="Qualcomm Ubuntu"></a>
  <a href="https://docs.ros.org/en/jazzy/" target="_blank"><img src="https://img.shields.io/badge/ROS%20Jazzy-1c428a?style=for-the-badge&logo=ros&logoColor=white" alt="Jazzy"></a>
  
</div>

---

## 👋 Overview

> 📌 **QRB ROS Video Package Features**
> - Hardware-accelerated H.264/H.265 video encoding and decoding using Qualcomm VPU
> - Zero-copy memory management for high-performance video processing
> - Seamless integration with ROS 2 ecosystem and camera pipelines
> - Support for real-time video streaming and file I/O operations

> 📌 **System Architecture**

```mermaid
flowchart LR
    %% Node definitions with custom styling
    CameraNode["Camera Node<br>(qrb_ros::camera::CameraNode)"]:::publisher
    ImageReader["ImageReader<br>(qrb_ros::video::ImageReader)"]:::publisher
    EncoderNode["Video Encoder Node<br>(qrb_ros::video::Encoder)"]:::pubsub
    CompressedWriter["CompressedWriter<br>(qrb_ros::video::CompressedWriter)"]:::subscriber
    CompressedReader["CompressedReader<br>(qrb_ros::video::CompressedReader)"]:::publisher
    DecoderNode["Video Decoder Node<br>(qrb_ros::video::Decoder)"]:::pubsub
    ImageWriter["ImageWriter<br>(qrb_ros::video::ImageWriter)"]:::subscriber
    DisplayNode["Display Node<br>(Optional)"]:::subscriber
    
    %% Topic connections
    CameraNode -- "Topic: qrb_ros::transport::type::Image" --> EncoderNode
    ImageReader -- "Topic: qrb_ros::transport::type::Image" --> EncoderNode
    EncoderNode -- "Topic: sensor_msgs::msg::CompressedImage" --> DecoderNode
    EncoderNode -- "Topic: sensor_msgs::msg::CompressedImage" --> CompressedWriter
    CompressedReader -- "Topic: sensor_msgs::msg::CompressedImage" --> DecoderNode
    DecoderNode -- "Topic: qrb_ros::transport::type::Image" --> ImageWriter
    DecoderNode -- "Topic: qrb_ros::transport::type::Image" --> DisplayNode
    
    %% Custom styling for nodes
    classDef publisher fill:#ff9900,stroke:#333,stroke-width:2px,color:#000
    classDef subscriber fill:#42d1f5,stroke:#333,stroke-width:2px,color:#000
    classDef pubsub fill:#c3ff00,stroke:#333,stroke-width:2px,color:#000
    classDef parameter fill:#dddddd,stroke:#333,stroke-width:2px,color:#000
```

> 📌 **Architecture Components:**
> - **Video Encoder Node**: Converts raw images to compressed H.264/H.265 streams using Qualcomm VPU hardware
> - **Video Decoder Node**: Decodes compressed video streams back to raw image frames
> - **Camera Support**: Seamlessly accepts input from QRB ROS Camera package with zero-copy transport
> - **File I/O Components**: CompressedWriter and ImageWriter for saving video data to files
> - **Hardware Acceleration**: Leverages Qualcomm Video Processing Unit (VPU) for efficient encoding/decoding
> - **Memory Management**: Utilizes DMA buffers and qrb_ros_transport for zero-copy operations

## 🔎 Table of Contents

  * [APIs](#-apis)
  * [Supported Targets](#-supported-targets)
  * [Installation](#-installation)
  * [Usage](#-usage)
  * [Build from Source](#-build-from-source)
  * [Contributing](#-contributing)
  * [License](#-license)

## ⚓ APIs

### 🔹 QRB ROS Video APIs

#### ROS Interfaces

**Topics: Encoder**

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `input` | `qrb_ros::transport::type::Image` | Uncompressed YUV frames |
| `output` | `sensor_msgs::msg::CompressedImage` | Compressed H.264/H.265 video stream |


**Topics: Decoder**

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `input` | `sensor_msgs::msg::CompressedImage` | Compressed H.264/H.265 video stream |
| `output` | `qrb_ros::transport::type::Image` | Decoded YUV frames |

#### ROS Parameters

**Video Encoder Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `format` | string | "h264" | Video codec format (h264/h265) |
| `pixel_format` | string | "nv12" | Input pixel format (nv12/p010) |
| `width` | int | 1920 | Video width |
| `height` | int | 1080 | Video height |
| `framerate` | int | 30 | Frames per second |
| `bitrate` | int | 5000000 | Target bitrate in bits/second |
| `rate_control` | string | "variable" | Rate control mode (variable/cbr) |
| `profile` | string | "main" | Codec profile (baseline/main/high) |
| `level` | string | "4.1" | Codec level |

**Video Decoder Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `format` | string | "h264" | Input codec format |
| `pixel-format` | string | "nv12" | Output pixel format |

## 🎯 Supported Targets

- **Ubuntu 24.04 LTS (Noble)**
- **ROS 2 Humble Humble and Jazzy**

Hardware Requirements:
- Qualcomm Video Processing Unit (VPU) for hardware acceleration
- Camera module compatible with qrb_ros_camera package (optional)

---

## ✨ Installation


## 🚀 Usage

Launch the video encoder for real-time video compression:

```bash
ros2 launch qrb_ros_video encoder_launch.py
```

Launch the video decoder for video decompression:

```bash
ros2 launch qrb_ros_video decoder_launch.py
```

**Example workflows:**

1. **Local Video File Recording:**
   ```bash
   # Terminal 1: Start video encoder
   ros2 launch qrb_ros_video encoder_launch.py
   ```

2. **Local Video File Playback:**
   ```bash
   # Terminal 1: Start video decoder
   ros2 launch qrb_ros_video decoder_launch.py
   ```
---

## 👨‍💻 Build from Source

Currently, we only support build with QCLINUX SDK.

1. Setup environments follow this document 's [Set up the cross-compile environment.](https://docs.qualcomm.com/bundle/publicresource/topics/80-65220-2/develop-your-first-application_6.html?product=1601111740013072&facet=Qualcomm%20Intelligent%20Robotics%20(QIRP)%20Product%20SDK&state=releasecandidate) part
2. Create `ros_ws` directory in `<qirp_decompressed_workspace>/qirp-sdk/`
3. Clone this repository under `<qirp_decompressed_workspace>/qirp-sdk/ros_ws`

    ```bash
    git clone https://github.com/qualcomm-qrb-ros/qrb_ros_imu.git
    git clone https://github.com/qualcomm-qrb-ros/lib_mem_dmabuf.git
    git clone https://github.com/qualcomm-qrb-ros/qrb_ros_transport.git
    git clone https://github.com/qualcomm-qrb-ros/qrb_ros_video.git
    ```

4. Build this project

   * For ROS2 Humble

   ```bash
   export AMENT_PREFIX_PATH="${OECORE_TARGET_SYSROOT}/usr;${OECORE_NATIVE_SYSROOT}/usr"
   export PYTHONPATH=${PYTHONPATH}:${OECORE_TARGET_SYSROOT}/usr/lib/python3.10/site-packages
   
   colcon build --merge-install --cmake-args \
     -DPython3_ROOT_DIR=${OECORE_TARGET_SYSROOT}/usr \
     -DPython3_NumPy_INCLUDE_DIR=${OECORE_TARGET_SYSROOT}/usr/lib/python3.10/site-packages/numpy/core/include \
     -DPYTHON_SOABI=cpython-310-aarch64-linux-gnu -DCMAKE_STAGING_PREFIX=$(pwd)/install \
     -DCMAKE_PREFIX_PATH=$(pwd)/install/share \
     -DBUILD_TESTING=OFF
   ```

   * For ROS2 Jazzy

   ```bash
   export AMENT_PREFIX_PATH="${OECORE_TARGET_SYSROOT}/usr;${OECORE_NATIVE_SYSROOT}/usr"
   export PYTHONPATH=${PYTHONPATH}:${OECORE_NATIVE_SYSROOT}/usr/lib/python3.12/site-packages

   colcon build --merge-install --cmake-args \
      -DPython3_ROOT_DIR=${OECORE_NATIVE_SYSROOT}/usr \
      -DPython3_NumPy_INCLUDE_DIR=${OECORE_NATIVE_SYSROOT}/usr/lib/python3.12/site-packages/numpy/core/include \
      -DSYSROOT_LIBDIR=${OECORE_TARGET_SYSROOT}/usr/lib \
      -DSYSROOT_INCDIR=${OECORE_TARGET_SYSROOT}/usr/include \
      -DPYTHON_SOABI=cpython-312-aarch64-linux-gnu -DCMAKE_STAGING_PREFIX=$(pwd)/install \
      -DCMAKE_PREFIX_PATH=$(pwd)/install/share \
      -DBUILD_TESTING=OFF
   ```

5. Push to the device & Install

   ```bash
   cd `<qirp_decompressed_workspace>/qirp-sdk/ros_ws/install`
   tar czvf qrb_ros_video.tar.gz lib share
   scp qrb_ros_video.tar.gz root@[ip-addr]:/opt/
   ssh root@[ip-addr]
   (ssh) tar -zxf /opt/qrb_ros_video.tar.gz -C /opt/qcom/qirp-sdk/usr/
   ```

## 🤝 Contributing

We love community contributions! Get started by reading our [CONTRIBUTING.md](CONTRIBUTING.md).  
Feel free to create an issue for bug reports, feature requests, or any discussion 💡.

## ❤️ Contributors

> 📌 **[Jean Xiao](jianxiao@qti.qualcomm.com)** 

## 📜 License

Project is licensed under the [BSD-3-clause License](https://spdx.org/licenses/BSD-3-Clause.html). See [LICENSE](./LICENSE) for the full license text.

