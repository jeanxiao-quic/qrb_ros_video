/*
**************************************************************************************************
* Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
**************************************************************************************************
*/

#include "GstreamerDecoder.hpp"

#include "Log.hpp"

namespace qrb::video_v4l2
{
std::shared_ptr<Client> GstreamerDecoder::create(const Format & compressed)
{
  auto decoder = std::make_shared<GstreamerDecoder>(compressed);
  auto client = std::make_shared<Client>();
  client->setChannelCB(decoder);
  decoder->setNotifier(client);
  return client;
}

std::string GstreamerDecoder::codecElementName() const
{
  return compressedFormat == Format::H264 ? "v4l2h264dec" : "v4l2h265dec";
}

std::string GstreamerDecoder::parserElementName() const
{
  return compressedFormat == Format::H264 ? "h264parse" : "h265parse";
}

GstCaps * GstreamerDecoder::buildSinkCaps() const
{
  const char * media = toGstMediaType(compressedFormat);
  if (media == nullptr) {
    LOGE("%s: no media type for compressed format %d", __PRETTY_FUNCTION__,
        static_cast<int>(compressedFormat));
    return nullptr;
  }

  return gst_caps_new_simple(media, "stream-format", G_TYPE_STRING, "byte-stream", "alignment",
      G_TYPE_STRING, "au", nullptr);
}

GstCaps * GstreamerDecoder::buildSrcCaps() const
{
  const char * drmFormat = nullptr;
  switch (settings.pixelFormat) {
    case Format::NV12:
      drmFormat = "NV12";
      break;
    case Format::NV12C:
      drmFormat = "NV12:0x0500000000000001";
      break;
    default:
      break;
  }
  if (drmFormat != nullptr) {
    auto * dmaCaps = gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING, "DMA_DRM",
        "drm-format", G_TYPE_STRING, drmFormat, nullptr);
    gst_caps_set_features_simple(dmaCaps, gst_caps_features_new("memory:DMABuf", nullptr));
    return dmaCaps;
  }

  const GstVideoFormat fmt = toGstVideoFormat(settings.pixelFormat);
  if (fmt == GST_VIDEO_FORMAT_UNKNOWN) {
    LOGE("%s: no GStreamer video format for pixel format %d", __PRETTY_FUNCTION__,
        static_cast<int>(settings.pixelFormat));
    return nullptr;
  }

  // Constrain the pixel format only. Width and height are left unfixed on purpose: this is
  // what lets v4l2h264dec renegotiate on a mid-stream resolution change without the
  // RECONFIGURING state machine the V4L2 backend needs. Each output sample carries its own
  // caps, which wrapSample() reads back.
  return gst_caps_new_simple(
      "video/x-raw", "format", G_TYPE_STRING, gst_video_format_to_string(fmt), nullptr);
}

void GstreamerDecoder::configureCodecElement(GstElement * codec) const
{
  // Compressed input is small and already copied once by the caller, so MMAP on the output
  // queue is fine and avoids constraining the producer's allocation. Capture must be dmabuf
  // so decoded frames leave as fds for zero-copy ROS transport.
  gst_util_set_object_arg(G_OBJECT(codec), "capture-io-mode", "dmabuf");
}
}  // namespace qrb::video_v4l2
