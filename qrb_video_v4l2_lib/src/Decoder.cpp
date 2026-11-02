/*
**************************************************************************************************
* Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
**************************************************************************************************
*/

#include "Decoder.hpp"

#include "Backend.hpp"
#include "Log.hpp"

#if QRB_VIDEO_HAS_V4L2
#include "V4l2Decoder.hpp"
#endif

#if QRB_VIDEO_HAS_GST
#include "GstreamerDecoder.hpp"
#endif

namespace qrb::video_v4l2
{
std::shared_ptr<Client> Decoder::create(std::string mime)
{
  return create(std::move(mime), defaultTarget());
}

std::shared_ptr<Client> Decoder::create(std::string mime, VideoCodec::Target target)
{
  Format f = mime == MIME_H264 ? Format::H264 : Format::HEVC;

  switch (target) {
    case VideoCodec::Target::V4l2:
#if QRB_VIDEO_HAS_V4L2
      return V4l2Decoder::create(f);
#else
      LOGE("%s: V4l2 backend was not built into this library", __func__);
      return nullptr;
#endif
    case VideoCodec::Target::Gst:
#if QRB_VIDEO_HAS_GST
      return GstreamerDecoder::create(f);
#else
      LOGE("%s: Gst backend was not built into this library", __func__);
      return nullptr;
#endif
  }

  LOGE("%s: unknown target %d", __func__, static_cast<int>(target));
  return nullptr;
}
}  // namespace qrb::video_v4l2
