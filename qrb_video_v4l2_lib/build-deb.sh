#!/bin/bash

# Build script for qrb_video_v4l2_lib debian package

set -e  # Exit on any error

echo "Building qrb_video_v4l2_lib debian package..."

# Clean previous builds
echo "Cleaning previous builds..."
rm -rf debian/tmp debian/.debhelper debian/libqrb-video-v4l2-*log
rm -rf debian/debhelper-build-stamp debian/debhelper-build-stamp debian/libqrb-video-v4l2-*substvars

# Build the package
echo "Building package..."
debuild -us -uc -b

echo "Build completed successfully!"
echo "Generated packages:"
ls -la ../*.deb 2>/dev/null || echo "No .deb files found in parent directory" 