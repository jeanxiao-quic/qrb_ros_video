/*
**************************************************************************************************
* Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
**************************************************************************************************
*/

#include "Backend.hpp"

// QRB_VIDEO_HAS_V4L2, QRB_VIDEO_HAS_GST and QRB_VIDEO_DEFAULT_TARGET_GST are set as
// PRIVATE compile definitions by CMake (always 0 or 1), so they are visible here but never
// in an installed header.
#if !QRB_VIDEO_HAS_V4L2 && !QRB_VIDEO_HAS_GST
#error "No backend enabled: set QRB_VIDEO_ENABLE_V4L2 and/or QRB_VIDEO_ENABLE_GST"
#endif

namespace qrb::video_v4l2
{
VideoCodec::Target defaultTarget()
{
#if QRB_VIDEO_DEFAULT_TARGET_GST
  return VideoCodec::Target::Gst;
#else
  return VideoCodec::Target::V4l2;
#endif
}

bool isAvailable(VideoCodec::Target target)
{
  switch (target) {
    case VideoCodec::Target::V4l2:
      return QRB_VIDEO_HAS_V4L2;
    case VideoCodec::Target::Gst:
      return QRB_VIDEO_HAS_GST;
  }
  return false;
}

const char * toString(VideoCodec::Target target)
{
  switch (target) {
    case VideoCodec::Target::V4l2:
      return "v4l2";
    case VideoCodec::Target::Gst:
      return "gst";
  }
  return "unknown";
}

bool fromString(const std::string & name, VideoCodec::Target & target)
{
  if (name == "v4l2") {
    target = VideoCodec::Target::V4l2;
    return true;
  }
  if (name == "gst") {
    target = VideoCodec::Target::Gst;
    return true;
  }
  return false;
}
}  // namespace qrb::video_v4l2
