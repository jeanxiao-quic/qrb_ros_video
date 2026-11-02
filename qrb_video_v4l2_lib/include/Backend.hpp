/*
**************************************************************************************************
* Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
**************************************************************************************************
*/

#ifndef QRB_VIDEO_V4L2__BACKEND_HPP_
#define QRB_VIDEO_V4L2__BACKEND_HPP_

#include <string>

#include "VideoCodec.hpp"

namespace qrb::video_v4l2
{
// Which backends were compiled in, and which one Encoder/Decoder::create(mime) picks, are
// build-time decisions (QRB_VIDEO_ENABLE_V4L2 / QRB_VIDEO_ENABLE_GST /
// QRB_VIDEO_DEFAULT_TARGET). These accessors exist so callers can query and report that
// without the defining macros leaking into installed headers.

// The target used by the single-argument Encoder/Decoder::create(). Always available.
VideoCodec::Target defaultTarget();

// Whether the given target was built into this library. Selecting an unavailable target
// makes create() log and return nullptr.
bool isAvailable(VideoCodec::Target target);

// "v4l2" or "gst". Never null.
const char * toString(VideoCodec::Target target);

// Parses "v4l2" / "gst" (case-sensitive). Leaves target untouched and returns false on an
// unrecognised name. Does not check availability -- use isAvailable() for that.
bool fromString(const std::string & name, VideoCodec::Target & target);
}  // namespace qrb::video_v4l2

#endif  // QRB_VIDEO_V4L2__BACKEND_HPP_
