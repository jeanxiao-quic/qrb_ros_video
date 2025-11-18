#!/bin/bash

# Build script for qrb_ros_video debian package

set -e  # Exit on any error

echo "Building qrb_ros_video debian package..."

# Clean previous builds
echo "Cleaning previous builds..."
rm -rf debian/tmp debian/.debhelper debian/ros-jazzy-qrb-ros-video*log
rm -rf debian/ros-jazzy-qrb-ros-video debian/debhelper-build-stamp debian/ros-jazzy-qrb-ros-video*substvars

# Build the package
echo "Building package..."
debuild -us -uc -b

echo "Build completed successfully!"
echo "Generated packages:"
ls -la ../*.deb 2>/dev/null || echo "No .deb files found in parent directory" 