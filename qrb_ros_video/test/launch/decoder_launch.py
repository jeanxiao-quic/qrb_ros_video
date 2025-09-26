# Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

import launch
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='decoder_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='qrb_ros_video',
                plugin='qrb_ros::video::Decoder',
                namespace='playback_ns',
                name='decoder_node',
                parameters=[{
                    'format': "h264",
                    'pixel-format': "nv12",
                }],
                extra_arguments=[{"use_intra_process_comms": True}],
                remappings=[("input", "decoder_node/input"),
                    ("output", "writer_node/raw_image")]
            ),
            ComposableNode(
                package='qrb_ros_video',
                plugin='qrb_ros::video::ImageWriter',
                namespace='playback_ns',
                name='writer_node',
                parameters=[{
                    'url': "/dev/null",
                    'format': "nv12",
                }],
                extra_arguments=[{"use_intra_process_comms": True}],
                remappings=[("input", "writer_node/raw_image")]
            ),
            ComposableNode(
                package='qrb_ros_video',
                plugin='qrb_ros::video::CompressedReader',
                namespace='playback_ns',
                name='reader_node',
                parameters=[{
                    'url': "/data/1920_1080.mp4",
                    'format': "mp4",
                    'width': "1920",
                    'height': "1080",
                    'framerate': "30",
                }],
                extra_arguments=[{"use_intra_process_comms": True}],
                remappings=[("output", "decoder_node/input")]
            ),
        ],
        output='screen',
    )

    return launch.LaunchDescription([container])
