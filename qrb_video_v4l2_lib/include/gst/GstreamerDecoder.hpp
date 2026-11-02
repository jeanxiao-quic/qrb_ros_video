/*
 **************************************************************************************************
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 **************************************************************************************************
 */

#ifndef QRB_VIDEO_V4L2__GSTREAMERDECODER_HPP_
#define QRB_VIDEO_V4L2__GSTREAMERDECODER_HPP_

#include "BufferChannel.hpp"
#include "GstreamerCodec.hpp"

namespace qrb::video_v4l2
{
class GstreamerDecoder : public GstreamerCodec
{
public:
  explicit GstreamerDecoder(const Format & compressed)
    : GstreamerCodec(CodecType::VideoDecoder, compressed)
  {
  }

  ~GstreamerDecoder() override = default;

  static std::shared_ptr<Client> create(const Format & compressed);

protected:
  std::string codecElementName() const override;

  std::string parserElementName() const override;

  GstCaps * buildSinkCaps() const override;

  GstCaps * buildSrcCaps() const override;

  Format outputFormat() const override { return settings.pixelFormat; }

  void configureCodecElement(GstElement * codec) const override;
};
}  // namespace qrb::video_v4l2

#endif  // QRB_VIDEO_V4L2__GSTREAMERDECODER_HPP_
