/*
**************************************************************************************************
* Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
**************************************************************************************************
*/

#include "V4l2Decoder.hpp"

namespace qrb::video_v4l2
{
std::shared_ptr<Client> V4l2Decoder::create(const Format & compressed)
{
  auto encoder = std::make_shared<V4l2Decoder>(compressed);
  auto client = std::make_shared<Client>();
  client->setChannelCB(encoder);
  encoder->setNotifier(client);
  return client;
}

bool V4l2Decoder::configure(const Setting & s)
{
  switch (s.type) {
    case Setting::BITRATE: {
      auto bitrate = std::any_cast<Bitrate>(s.data);
      auto setting = get<V4l2Bitrate>(INPUT_PORT);
      *setting = bitrate;
    } break;
    case Setting::FORMAT: {
      auto fmt = std::any_cast<Format>(s.data);
      const auto f = get<V4l2Format>(OUTPUT_PORT);
      *f = fmt;
    } break;
    case Setting::RESOLUTION: {
      auto resolution = std::any_cast<Resolution>(s.data);
      auto r = get<V4l2Format>(OUTPUT_PORT);
      *r = resolution;
    } break;
    default:
      return false;
  }
  return true;
}

bool V4l2Decoder::start()
{
  setCodecFormat();
  getDriver()->subscribeEvent(V4L2_EVENT_SOURCE_CHANGE);
  getDriver()->subscribeEvent(V4L2_EVENT_EOS);
  v4l2_decoder_cmd cmd = {};
  cmd.cmd = V4L2_DEC_CMD_START;
  driver->decCommand(&cmd) == 0;
  return V4l2Codec::start();
}

bool V4l2Decoder::stop()
{
  v4l2_decoder_cmd cmd = {};
  getDriver()->unsubscribeEvent(V4L2_EVENT_SOURCE_CHANGE);
  getDriver()->unsubscribeEvent(V4L2_EVENT_EOS);
  cmd.cmd = V4L2_DEC_CMD_STOP;
  driver->decCommand(&cmd);
  return V4l2Codec::stop();
}

void V4l2Decoder::setCodecFormat()
{
  auto setting = get<V4l2Format>(INPUT_PORT);
  *setting = compressedFormat;
}

bool V4l2Decoder::reconfigurePort(bool port)
{
  emptied_.get_future().get();
  if (port == OUTPUT_PORT) {
    bool ret = reconfigureOutput();
    if (ret) {
      // stopStreaming(INPUT_PORT);
      // {
      //   std::lock_guard<std::mutex> l(mutex_);
      //   buffer_queued_[INPUT_PORT].clear();
      // }
      // startStreaming(INPUT_PORT);
      // v4l2_decoder_cmd cmd = {};
      // cmd.cmd = V4L2_DEC_CMD_START;
      // getDriver()->decCommand(&cmd);
      drainPendingInput();
    }
    emptied_ = std::promise<bool>();
    return ret;
  }
  return false;
}

bool V4l2Decoder::reconfigureOutput()
{
  get<V4l2Format>(OUTPUT_PORT)->get();
  prepareBufferPool<DmabufAllocator>(OUTPUT_PORT);
  startStreaming(OUTPUT_PORT);
  state = STARTED;
  for (int i = 0; i < 4; i++) {
    if (!feedOutputBuffer()) {
      break;
    }
  }
  return true;
}

bool V4l2Decoder::drain()
{
  v4l2_decoder_cmd cmd = {};
  cmd.cmd = V4L2_DEC_CMD_STOP;
  return driver->decCommand(&cmd) == 0;
}
}  // namespace qrb::video_v4l2
