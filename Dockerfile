FROM library/ros:jazzy-ros-base

RUN add-apt-repository -y ppa:ubuntu-qcom-iot/qcom-ppa && \
    add-apt-repository -y ppa:ubuntu-qcom-iot/qirp

RUN apt-get update && apt-get install -y --no-install-recommends \
    cmake \
    pkg-config \
    ros-jazzy-std-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-rclcpp \
    ros-jazzy-rclcpp-components \
    ros-jazzy-ament-cmake-auto \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    ros-jazzy-qrb-ros-transport-image-type libqrb-video-v4l2-dev \
    devscripts sbuild fakeroot debhelper-compat software-properties-common \
    && rm -rf /var/lib/apt/lists/*
