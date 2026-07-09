/*
**************************************************************************************************
* Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
**************************************************************************************************
*/

#ifndef QRB_VIDEO_V4L2__V4L2DECODER_HPP_
#define QRB_VIDEO_V4L2__V4L2DECODER_HPP_

#include "V4l2Codec.hpp"

namespace qrb::video_v4l2
{
class V4l2Decoder : public V4l2Codec
{
public:
  V4l2Decoder(const Format & compressed) : V4l2Codec(CodecType::VideoDecoder, "/dev/video32")
  {
    compressedFormat = compressed;
  };
  ~V4l2Decoder() override = default;

  static std::shared_ptr<Client> create(const Format & compressed);

  bool configure(const Setting & s) override;

  bool start() override;

  bool stop() override;

protected:
  void setCodecFormat();

  bool startPort(bool input) override;

  bool reconfigurePort(bool input) override;

  bool drain() override;

  bool outputPortStarted = false;
};

}  // namespace qrb::video_v4l2

#endif  // QRB_VIDEO_V4L2__V4L2DECODER_HPP_
