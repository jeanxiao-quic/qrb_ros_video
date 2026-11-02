/*
 **************************************************************************************************
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 **************************************************************************************************
 */

#ifndef QRB_VIDEO_V4L2__GSTREAMERENCODER_HPP_
#define QRB_VIDEO_V4L2__GSTREAMERENCODER_HPP_

#include "BufferChannel.hpp"
#include "GstreamerCodec.hpp"

namespace qrb::video_v4l2
{
class GstreamerEncoder : public GstreamerCodec
{
public:
  explicit GstreamerEncoder(const Format & compressed)
    : GstreamerCodec(CodecType::VideoEncoder, compressed)
  {
  }

  ~GstreamerEncoder() override = default;

  static std::shared_ptr<Client> create(const Format & compressed);

protected:
  std::string codecElementName() const override;

  GstCaps * buildSinkCaps() const override;

  GstCaps * buildSrcCaps() const override;

  Format outputFormat() const override { return compressedFormat; }

  void configureCodecElement(GstElement * codec) const override;
};
}  // namespace qrb::video_v4l2

#endif  // QRB_VIDEO_V4L2__GSTREAMERENCODER_HPP_
