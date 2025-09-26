# Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

import launch
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
        name='encoder_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='qrb_ros_video',
                plugin='qrb_ros::video::Encoder',
                namespace='recording_ns',
                name='encoder_node',
                parameters=[{
                    'format': "h264",
                    'pixel-format': "nv12",
                    'width': "1920",
                    'height': "1080",
                    'framerate': "30",
                    'bitrate': "20000000",
                    'rate-control': "variable",
                    'profile': "high",
                    'level': "4.1",
                }],
                extra_arguments=[{"use_intra_process_comms": True}],
                remappings=[("input", "encoder_node/input"),
                            ("output", "writer_node/compressed_image")
                            ]
            ),
            ComposableNode(
                package='qrb_ros_video',
                plugin='qrb_ros::video::CompressedWriter',
                namespace='recording_ns',
                name='writer_node',
                parameters=[{
                    'url': "/data/1920_1080_nv12.mp4",
                    'format': "mp4",
                }],
                extra_arguments=[{"use_intra_process_comms": True}],
                remappings=[("input", "writer_node/compressed_image"),
                            ]
            ),
            ComposableNode(
                package='qrb_ros_video',
                plugin='qrb_ros::video::ImageReader',
                namespace='recording_ns',
                name='reader_node',
                parameters=[{
                    'url': "/data/1920_1080_nv12.yuv",
                    'format': "nv12",
                    'width': "1920",
                    'height': "1080",
                    'framerate': "30",
                }],
                extra_arguments=[{"use_intra_process_comms": True}],
                remappings=[("output", "encoder_node/input")]
            ),
        ],
        output='screen',
    )

    return launch.LaunchDescription([container])
