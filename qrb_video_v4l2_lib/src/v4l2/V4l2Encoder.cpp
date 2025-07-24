/*
**************************************************************************************************
* Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
**************************************************************************************************
*/

#include "V4l2Encoder.hpp"

#include "Memory.hpp"

namespace qrb::video_v4l2
{
std::shared_ptr<Client> V4l2Encoder::create(const Format & compressed)
{
  auto encoder = std::make_shared<V4l2Encoder>(compressed);
  auto client = std::make_shared<Client>();
  client->setChannelCB(encoder);
  encoder->setNotifier(client);
  return client;
}

bool V4l2Encoder::configure(const Setting & s)
{
  switch (s.type) {
    case Setting::PROFILE: {
      auto profile = std::any_cast<Profile>(s.data);
      auto setting = get<V4l2Profile>(OUTPUT_PORT);
      *setting = profile;
    } break;
    case Setting::LEVEL: {
      auto level = std::any_cast<Level>(s.data);
      auto setting = get<V4l2Level>(OUTPUT_PORT);
      *setting = level;
    } break;
    case Setting::BITRATE: {
      auto bitrate = std::any_cast<Bitrate>(s.data);
      auto setting = get<V4l2Bitrate>(OUTPUT_PORT);
      *setting = bitrate;
    } break;
    case Setting::FORMAT: {
      auto fmt = std::any_cast<Format>(s.data);
      const auto f = get<V4l2Format>(INPUT_PORT);
      *f = fmt;
    } break;
    case Setting::RESOLUTION: {
      auto resolution = std::any_cast<Resolution>(s.data);
      auto r = get<V4l2Format>(s.type);
      *r = resolution;
    } break;
    case Setting::FRAMERATE: {
      auto framerate = std::any_cast<Framerate>(s.data);
      auto setting = get<V4l2Framerate>(OUTPUT_PORT);
      *setting = framerate;
    } break;
  }
  return V4l2Codec::configure(s);
}

bool V4l2Encoder::start()
{
  setCodecFormat();
  v4l2_encoder_cmd cmd = {};
  cmd.cmd = V4L2_ENC_CMD_START;
  driver->encCommand(&cmd) == 0;

  v4l2_control ctrl = {};
  ctrl.id = V4L2_CID_MPEG_VIDC_VUI_TIMING_INFO;
  ctrl.value = 1;  // Enable VUI timing info
  driver->setControl(&ctrl);
  return V4l2Codec::start();
}

bool V4l2Encoder::stop()
{
  v4l2_encoder_cmd cmd = {};
  cmd.cmd = V4L2_ENC_CMD_STOP;
  driver->encCommand(&cmd);
  return V4l2Codec::stop();
}

void V4l2Encoder::setCodecFormat()
{
  auto setting = get<V4l2Format>(OUTPUT_PORT);
  *setting = compressedFormat;
}

bool V4l2Encoder::drain()
{
  v4l2_encoder_cmd cmd = {};
  cmd.cmd = V4L2_ENC_CMD_STOP;
  return driver->encCommand(&cmd) == 0;
}
}  // namespace qrb::video_v4l2
