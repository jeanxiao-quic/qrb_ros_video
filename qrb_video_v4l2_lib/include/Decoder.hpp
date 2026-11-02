/*
 **************************************************************************************************
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 **************************************************************************************************
 */

#ifndef QRB_VIDEO_V4L2__VIDEODECODER_HPP_
#define QRB_VIDEO_V4L2__VIDEODECODER_HPP_

#include "BufferChannel.hpp"

namespace qrb::video_v4l2
{
class Decoder : public VideoCodec
{
public:
  Decoder() = default;
  ~Decoder() override = default;

  static std::shared_ptr<Client> create(std::string mime);

  static std::shared_ptr<Client> create(std::string mime, VideoCodec::Target target);

private:
  std::weak_ptr<Client> interface;
};
}  // namespace qrb::video_v4l2
#endif  // QRB_VIDEO_V4L2__VIDEODECODER_HPP_
