/*
**************************************************************************************************
* Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
**************************************************************************************************
*/

#include "V4l2Codec.hpp"

#include <sys/ioctl.h>

#include <algorithm>
#include <cstring>

#include "Encoder.hpp"
#include "Log.hpp"

namespace qrb::video_v4l2
{
const std::unordered_map<uint32_t, int32_t> V4l2Codec::avcProfileMapping = {
  { Profile::AVC_BASELINE, V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE },
  { Profile::AVC_CONSTRAINED_BASELINE, V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE },
  { Profile::AVC_MAIN, V4L2_MPEG_VIDEO_H264_PROFILE_MAIN },
  { Profile::AVC_HIGH, V4L2_MPEG_VIDEO_H264_PROFILE_HIGH },
  { Profile::AVC_CONSTRAINED_HIGH, V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_HIGH },
};

const std::unordered_map<uint32_t, int32_t> V4l2Codec::hevcProfileMapping = {
  { Profile::HEVC_MAIN, V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN },
  { Profile::HEVC_MAIN10, V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_10 },
  { Profile::HEVC_MAIN_STILL, V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_STILL_PICTURE },
};

const std::unordered_map<uint32_t, int32_t> V4l2Codec::avcLevelMapping = {
  { Level::AVC_1_0, V4L2_MPEG_VIDEO_H264_LEVEL_1_0 },
  { Level::AVC_1_B, V4L2_MPEG_VIDEO_H264_LEVEL_1B },
  { Level::AVC_1_1, V4L2_MPEG_VIDEO_H264_LEVEL_1_1 },
  { Level::AVC_1_2, V4L2_MPEG_VIDEO_H264_LEVEL_1_2 },
  { Level::AVC_1_3, V4L2_MPEG_VIDEO_H264_LEVEL_1_3 },
  { Level::AVC_2_0, V4L2_MPEG_VIDEO_H264_LEVEL_2_0 },
  { Level::AVC_2_1, V4L2_MPEG_VIDEO_H264_LEVEL_2_1 },
  { Level::AVC_2_2, V4L2_MPEG_VIDEO_H264_LEVEL_2_2 },
  { Level::AVC_3_0, V4L2_MPEG_VIDEO_H264_LEVEL_3_0 },
  { Level::AVC_3_1, V4L2_MPEG_VIDEO_H264_LEVEL_3_1 },
  { Level::AVC_3_2, V4L2_MPEG_VIDEO_H264_LEVEL_3_2 },
  { Level::AVC_4_0, V4L2_MPEG_VIDEO_H264_LEVEL_4_0 },
  { Level::AVC_4_1, V4L2_MPEG_VIDEO_H264_LEVEL_4_1 },
  { Level::AVC_4_2, V4L2_MPEG_VIDEO_H264_LEVEL_4_2 },
  { Level::AVC_5_0, V4L2_MPEG_VIDEO_H264_LEVEL_5_0 },
  { Level::AVC_5_1, V4L2_MPEG_VIDEO_H264_LEVEL_5_1 },
  { Level::AVC_5_2, V4L2_MPEG_VIDEO_H264_LEVEL_5_2 },
  { Level::AVC_6_0, V4L2_MPEG_VIDEO_H264_LEVEL_6_0 },
  { Level::AVC_6_1, V4L2_MPEG_VIDEO_H264_LEVEL_6_1 },
  { Level::AVC_6_2, V4L2_MPEG_VIDEO_H264_LEVEL_6_2 },
};

const std::unordered_map<uint32_t, int32_t> V4l2Codec::hevcLevelMapping = {
  { Level::HEVC_1_0, V4L2_MPEG_VIDEO_HEVC_LEVEL_1 },
  { Level::HEVC_2_0, V4L2_MPEG_VIDEO_HEVC_LEVEL_2 },
  { Level::HEVC_2_1, V4L2_MPEG_VIDEO_HEVC_LEVEL_2_1 },
  { Level::HEVC_3_0, V4L2_MPEG_VIDEO_HEVC_LEVEL_3 },
  { Level::HEVC_3_1, V4L2_MPEG_VIDEO_HEVC_LEVEL_3_1 },
  { Level::HEVC_4_0, V4L2_MPEG_VIDEO_HEVC_LEVEL_4 },
  { Level::HEVC_4_1, V4L2_MPEG_VIDEO_HEVC_LEVEL_4_1 },
  { Level::HEVC_5_0, V4L2_MPEG_VIDEO_HEVC_LEVEL_5 },
  { Level::HEVC_5_1, V4L2_MPEG_VIDEO_HEVC_LEVEL_5_1 },
  { Level::HEVC_5_2, V4L2_MPEG_VIDEO_HEVC_LEVEL_5_2 },
  { Level::HEVC_6_0, V4L2_MPEG_VIDEO_HEVC_LEVEL_6 },
  { Level::HEVC_6_1, V4L2_MPEG_VIDEO_HEVC_LEVEL_6_1 },
  { Level::HEVC_6_2, V4L2_MPEG_VIDEO_HEVC_LEVEL_6_2 },
};

const std::unordered_map<Bitrate::Mode, int32_t> V4l2Codec::bitrateModeMapping = {
  { Bitrate::CBR, V4L2_MPEG_VIDEO_BITRATE_MODE_CBR },
  { Bitrate::VBR, V4L2_MPEG_VIDEO_BITRATE_MODE_VBR },
};

const std::unordered_map<Format, uint32_t> V4l2Codec::formatMapping = {
  { Format::NV12, V4L2_PIX_FMT_NV12 },
  { Format::NV12C, V4L2_PIX_FMT_NV12C },
  { Format::P010, V4L2_PIX_FMT_P010 },
  { Format::TP10C, V4L2_PIX_FMT_TP10C },
  { Format::H264, V4L2_PIX_FMT_H264 },
  { Format::HEVC, V4L2_PIX_FMT_HEVC },
};

V4l2Codec::V4l2Codec(const CodecType type, std::string name) : type(type), state(State::STOPPED)
{
  driver = std::make_shared<V4l2Driver>(name);
  init();
}

void V4l2Codec::init()
{
  v4l2_capability caps = {};
  if (getDriver()->queryCapabilities(&caps) < 0 ||
      not(caps.capabilities & V4L2_CAP_VIDEO_M2M_MPLANE) ||
      not(caps.capabilities & V4L2_CAP_STREAMING)) {
  }
}

template <typename AllocatorType>
void V4l2Codec::prepareBufferPool(bool input)
{
  auto direction = input ? INPUT_PORT : OUTPUT_PORT;
  auto & pool = input ? inputPool : outputPool;
  get<V4l2BufferCount>(direction)->set();
  auto view = get<V4l2Format>(direction)->operator MemoryView();
  auto allocator = std::make_shared<AllocatorType>();
  allocator->set(view);
  pool = std::make_shared<BufferPool>();
  pool->registerAllocator(allocator);
}

bool V4l2Codec::configure(const Setting & s)
{
  LOGI("%s: configuring type %d", __PRETTY_FUNCTION__, s.type);
  return true;
}

bool V4l2Codec::start()
{
  if (state == STOPPED) {
    populateSettings();
    prepareBufferPool<DmabufAllocator>(INPUT_PORT);
    prepareBufferPool<DmabufAllocator>(OUTPUT_PORT);
    getDriver()->registerCallbacks(this);
    getDriver()->start();
    if (auto client_handler = notifier.lock()) {
      handler_ = std::make_shared<EventHandler>(
          shared_from_this(), std::dynamic_pointer_cast<Handler>(client_handler).get());
    }

    startStreaming(INPUT_PORT);
    startStreaming(OUTPUT_PORT);
    state = STARTED;
  } else {
    LOGE("V4l2Codec::start() called with invalid state %d", state.load());
  }
  return true;
}

bool V4l2Codec::stop()
{
  if (state == STARTED) {
    flush(OUTPUT_PORT);
    flush(INPUT_PORT);
    handler_ = nullptr;
    getDriver()->stop();
    state = STOPPED;
  }
  return true;
}

bool V4l2Codec::seek()
{
  return false;
}

bool V4l2Codec::pause()
{
  return false;
}

bool V4l2Codec::flush(bool input)
{
  if (state == STARTED || state == RECONFIGURING) {
    State old_state = state;
    state = FLUSHING;
    auto index = input ? INPUT_PORT : OUTPUT_PORT;
    stopStreaming(index);
    state = old_state;
  }
  return true;
}

bool V4l2Codec::drain()
{
  return false;
}

bool V4l2Codec::reconfigurePort(bool port)
{
  return true;
}

bool V4l2Codec::startStreaming(bool port)
{
  bool ret = false;
  ret = getDriver()->streamOn(port == INPUT_PORT ? INPUT_MPLANE : OUTPUT_MPLANE);
  return ret;
}

bool V4l2Codec::stopStreaming(bool port)
{
  bool ret = false;
  ret = getDriver()->streamOff(port == INPUT_PORT ? INPUT_MPLANE : OUTPUT_MPLANE);
  return ret;
}

bool V4l2Codec::onV4l2BufferDone(v4l2_buffer * buffer)
{
  bool ret = false;
  std::unique_lock lk(mutex_);
  auto index = buffer->type == OUTPUT_MPLANE ? OUTPUT_PORT : INPUT_PORT;
  LOGI("%s: received type %d done buffer", __PRETTY_FUNCTION__, buffer->type);
  auto it = buffer_queued_[index].find(buffer->index);
  std::shared_ptr<V4l2Buffer> doneBuffer = nullptr;
  if (it != buffer_queued_[index].end()) {
    doneBuffer = it->second;
    buffer_queued_[index].erase(it);
    if (state == RECONFIGURING || state == FLUSHING) {
      // Queue buffers all returned back
      if (buffer->flags & V4L2_BUF_FLAG_LAST) {
        LOGI("V4L2_BUF_FLAG_LAST received on index %d", index);
        flush(index);
        if (!buffer_queued_[index].empty()) {
          LOGI("pending queued buffer %d in driver, release all", buffer_queued_[index].size());
          buffer_queued_[index].clear();
        }
        emptied_.set_value(true);
      }
      return true;
    }
  } else {
    LOGE("%s: unfound buffer with index %d type %d", __PRETTY_FUNCTION__, buffer->index,
        buffer->type);
    return false;
  }
  lk.unlock();
  if (index == OUTPUT_PORT && buffer->flags & V4L2_BUF_FLAG_CODECCONFIG) {
    feedOutputBuffer();
  }
  if (index == OUTPUT_PORT && state == STARTED) {
    prepareForDispatch(doneBuffer, buffer);
    ret = dispatchBuffer(doneBuffer);
  }

  return ret;
}

bool V4l2Codec::onV4l2EventDone(v4l2_event * event)
{
  LOGI("onV4l2EventDone: received type %d", event->type);
  if (event->type == V4L2_EVENT_SOURCE_CHANGE &&
      event->u.src_change.changes == V4L2_EVENT_SRC_CH_RESOLUTION) {
    state = RECONFIGURING;
    auto msg = handler_->obtainMessage(MSG_RECONFIGURE_PORT);
    msg->data = OUTPUT_PORT;
    handler_->sendMessageAsync(msg);
  }
  return true;
}

bool V4l2Codec::onV4l2Error(error_t error)
{
  return true;
}

std::shared_ptr<Buffer> V4l2Codec::acquireBuffer()
{
  auto buffer = acquireInputBuffer();
  return buffer;
}

bool V4l2Codec::queueBuffer(const std::shared_ptr<Buffer> & item)
{
  if (item->isEOS()) {
    LOGI("Drain Codec");
    drain();
  } else {
    queueInputBuffer(item);
  }
  bool ret = feedOutputBuffer();
  return ret;
}

bool V4l2Codec::dispatchBuffer(const std::shared_ptr<Buffer> & item)
{
  const auto callback = notifier.lock();
  bool ret = false;
  if (callback) {
    LOGI("%s: buffer index %d", __PRETTY_FUNCTION__, item->index);
    ret = callback->onBufferAvailable(item);
  }
  return ret;
}

bool V4l2Codec::prepareForDispatch(std::shared_ptr<V4l2Buffer> & buf, v4l2_buffer * vb)
{
  buf->sequence = vb->sequence;
  buf->timestamp = vb->timestamp;
  buf->bytesused = vb->m.planes[0].bytesused;
  buf->offset = vb->m.planes[0].data_offset;
  if (vb->flags & V4L2_BUF_FLAG_LAST) {
    LOGI("Output buffer EOS received");
    buf->setEOS();
  }
  return true;
}

bool V4l2Codec::onAcquireBuffer(std::shared_ptr<Buffer> & item)
{
  item = acquireBuffer();
  return true;
}

bool V4l2Codec::onQueueBuffer(const std::shared_ptr<Buffer> & item)
{
  return queueBuffer(item);
}

bool V4l2Codec::onConfigure(const Setting & s)
{
  return configure(s);
}

bool V4l2Codec::onStart()
{
  return start();
}

bool V4l2Codec::onStop()
{
  return stop();
}

bool V4l2Codec::onPause()
{
  return pause();
}

bool V4l2Codec::onSeek()
{
  return seek();
}

void V4l2Codec::populateSettings() const
{
  std::for_each(settings.begin(), settings.end(), [](const auto & s) {
    auto input = s.second.first;
    if (input->dirty) {
      input->set();
    }
    auto output = s.second.second;
    if (output->dirty) {
      output->set();
    }
  });
}

std::shared_ptr<Buffer> V4l2Codec::acquireInputBuffer() const
{
  std::shared_ptr<Buffer> buffer;
  inputPool->acquire(buffer);
  return buffer;
}

std::shared_ptr<Buffer> V4l2Codec::acquireOutputBuffer() const
{
  std::shared_ptr<Buffer> buffer;
  outputPool->acquire(buffer);
  return buffer;
}

bool EventHandler::handleMessage(const std::shared_ptr<Message> & msg)
{
  bool ret = false;
  switch (msg->what) {
    case V4l2Codec::MSG_FLUSH_PORT: {
      bool direction = std::any_cast<bool>(msg->data);
      ret = self->flush(direction);
    } break;
    case V4l2Codec::MSG_RECONFIGURE_PORT: {
      bool direction = std::any_cast<bool>(msg->data);
      ret = self->reconfigurePort(direction);
    } break;
    case V4l2Codec::MSG_FEED_OUTPUT_BUFFER: {
      ret = self->feedOutputBuffer();
    } break;
    default:
      LOGE("%s: Unsupported message type %d received", __PRETTY_FUNCTION__, msg->what);
  }
  return finishMessage(msg, ret);
}

bool V4l2Codec::queueInputBuffer(const std::shared_ptr<Buffer> & item)
{
  int ret = 0;
  auto buffer = std::dynamic_pointer_cast<V4l2Buffer>(item);
  if (not buffer) {
    buffer = std::shared_ptr<V4l2Buffer>(
        new V4l2Buffer(*item), [backend = item](V4l2Buffer * buffer) { delete buffer; });
  }
  if (not buffer || state == FLUSHING) {
    LOGI(
        "%s: failed with buffer %p state %d", __PRETTY_FUNCTION__, buffer, static_cast<int>(state));
    return false;
  }
  v4l2_buffer buf = buffer->operator v4l2_buffer();
  buf.type = INPUT_MPLANE;
  std::lock_guard l(mutex_);
  if (buffer_queued_[INPUT_PORT].find(buf.index) != buffer_queued_[INPUT_PORT].end()) {
    buf.index = buffer_queued_[INPUT_PORT].size();
  }
  ret = getDriver()->queueBuf(&buf);
  if (ret == 0) {
    LOGI("%s: queued index %d buffer type %d", __PRETTY_FUNCTION__, buf.index, buf.type);
    buffer_queued_[INPUT_PORT][buf.index] = buffer;
  } else {
    LOGE("%s: failed to queue buffer %d buffer type %d", __PRETTY_FUNCTION__, buf.index, buf.type);
  }

  return ret == 0;
}

bool V4l2Codec::queueOutputBuffer(const std::shared_ptr<Buffer> & item)
{
  int ret = 0;
  auto buffer = std::dynamic_pointer_cast<V4l2Buffer>(item);
  if (not buffer) {
    buffer = std::shared_ptr<V4l2Buffer>(
        new V4l2Buffer(*item), [backend = item](V4l2Buffer * buffer) { delete buffer; });
  }
  if (not buffer || state == FLUSHING) {
    // A buffer from external source
    LOGI(
        "%s: failed with buffer %p state %d", __PRETTY_FUNCTION__, buffer, static_cast<int>(state));
    return false;
  }
  v4l2_buffer buf = buffer->operator v4l2_buffer();
  buf.type = OUTPUT_MPLANE;
  std::lock_guard l(mutex_);
  if (buffer_queued_[OUTPUT_PORT].find(buf.index) != buffer_queued_[OUTPUT_PORT].end()) {
    buf.index = buffer_queued_[OUTPUT_PORT].size();
  }
  ret = getDriver()->queueBuf(&buf);
  if (ret == 0) {
    LOGI("%s: queued index %d buffer type %d", __PRETTY_FUNCTION__, buf.index, buf.type);
    buffer_queued_[OUTPUT_PORT][buf.index] = buffer;
  } else {
    LOGE("%s: failed to queue buffer %d buffer type %d", __PRETTY_FUNCTION__, buf.index, buf.type);
  }
  return ret == 0;
}

bool V4l2Codec::feedOutputBuffer()
{
  bool result = false;
  std::shared_ptr<Buffer> buffer;
  outputPool->acquire(buffer);
  if (buffer) {
    result = queueOutputBuffer(buffer);
  }
  return result;
}

V4l2Codec::V4l2Profile::V4l2Profile(const std::shared_ptr<V4l2Codec> & instance, bool direction)
  : Impl(instance, direction)
{
  auto codec = getCodec();
  auto driver = getDriver();
  auto id = codec->compressedFormat == Format::H264 ? V4L2_CID_MPEG_VIDEO_H264_PROFILE :
                                                      V4L2_CID_MPEG_VIDEO_HEVC_PROFILE;
  if (codec && driver) {
    v4l2_queryctrl ctrl = {};
    ctrl.id = id;
    driver->queryControl(&ctrl);

    auto mapping = codec->compressedFormat == Format::H264 ? avcProfileMapping : hevcProfileMapping;
    std::for_each(mapping.begin(), mapping.end(), [&](const auto & p) {
      v4l2_querymenu querymenu = {};
      querymenu.id = id;
      querymenu.index = p.second;
      if (driver->queryMenu(&querymenu) == 0) {
        supportedProfile.push_back(p.first);
        if (ctrl.default_value == p.second) {
          value = p.first;
        }
      }
    });
  }
}

V4l2Codec::V4l2Profile & V4l2Codec::V4l2Profile::operator=(Profile & p)
{
  if (std::find(supportedProfile.begin(), supportedProfile.end(), p.value) !=
      supportedProfile.end()) {
    value = p.value;
    dirty = true;
  }
  return *this;
}

bool V4l2Codec::V4l2Profile::get()
{
  auto driver = getDriver();
  auto codec = getCodec();
  if (codec && driver) {
    uint32_t profle = codec->compressedFormat == Format::H264 ? V4L2_CID_MPEG_VIDEO_H264_PROFILE :
                                                                V4L2_CID_MPEG_VIDEO_HEVC_PROFILE;
    v4l2_control control = { profle, 0 };
    if (driver->getControl(&control) < 0)
      return false;
    auto mapping = codec->compressedFormat == Format::H264 ? avcProfileMapping : hevcProfileMapping;
    auto it = std::find_if(mapping.begin(), mapping.end(),
        [&control](const auto & p) { return p.second == control.value; });
    if (it == mapping.end())
      return false;
    value = it->first;
    return true;
  }
  return false;
}

bool V4l2Codec::V4l2Profile::set()
{
  bool ret = false;
  auto codec = getCodec();
  auto driver = getDriver();
  if (codec && driver) {
    auto profileMapping =
        codec->compressedFormat == Format::H264 ? avcProfileMapping : hevcProfileMapping;
    if (profileMapping.find(value) != profileMapping.end()) {
      uint32_t ctrlName = codec->compressedFormat == Format::H264 ?
                              V4L2_CID_MPEG_VIDEO_H264_PROFILE :
                              V4L2_CID_MPEG_VIDEO_HEVC_PROFILE;
      v4l2_control ctrl = { ctrlName, static_cast<int32_t>(profileMapping.at(value)) };
      driver->setControl(&ctrl);
      ret = true;
      dirty = false;
    }
  }
  return ret;
}

V4l2Codec::V4l2Level::V4l2Level(const std::shared_ptr<V4l2Codec> & instance, bool direction)
  : Impl(instance, direction)
{
  auto driver = getDriver();
  auto codec = getCodec();
  auto id = codec->compressedFormat == Format::H264 ? V4L2_CID_MPEG_VIDEO_H264_LEVEL :
                                                      V4L2_CID_MPEG_VIDEO_HEVC_LEVEL;
  auto mapping = codec->compressedFormat == Format::H264 ? avcLevelMapping : hevcLevelMapping;
  if (driver) {
    v4l2_queryctrl ctrl = {};
    ctrl.id = id;
    driver->queryControl(&ctrl);

    std::for_each(mapping.begin(), mapping.end(), [&](const auto & p) {
      v4l2_querymenu querymenu = {};
      querymenu.id = id;
      querymenu.index = p.second;
      if (driver->queryMenu(&querymenu) == 0) {
        supportedLevel.push_back(p.first);
        if (ctrl.default_value == p.second) {
          value = p.first;
        }
      }
    });
  }
}

V4l2Codec::V4l2Level & V4l2Codec::V4l2Level::operator=(Level & l)
{
  if (std::find(supportedLevel.begin(), supportedLevel.end(), l.value) != supportedLevel.end()) {
    value = l.value;
    dirty = true;
  }
  return *this;
}

bool V4l2Codec::V4l2Level::get()
{
  auto driver = getDriver();
  auto codec = getCodec();
  if (codec && driver) {
    uint32_t level = codec->compressedFormat == Format::H264 ? V4L2_CID_MPEG_VIDEO_H264_LEVEL :
                                                               V4L2_CID_MPEG_VIDEO_HEVC_LEVEL;
    v4l2_control control = { level, 0 };
    if (driver->getControl(&control) < 0)
      return false;
    auto mapping = codec->compressedFormat == Format::H264 ? avcLevelMapping : hevcLevelMapping;
    auto it = std::find_if(mapping.begin(), mapping.end(),
        [&control](const auto & p) { return p.second == control.value; });
    if (it == mapping.end())
      return false;
    value = it->first;
    return true;
  }
  return false;
}

bool V4l2Codec::V4l2Level::set()
{
  bool ret = false;
  auto codec = getCodec();
  auto driver = getDriver();
  if (codec && driver) {
    auto mapping = codec->compressedFormat == Format::H264 ? avcLevelMapping : hevcLevelMapping;
    if (mapping.find(value) != mapping.end()) {
      uint32_t ctrlName = codec->compressedFormat == Format::H264 ? V4L2_CID_MPEG_VIDEO_H264_LEVEL :
                                                                    V4L2_CID_MPEG_VIDEO_HEVC_LEVEL;
      v4l2_control ctrl = { ctrlName, static_cast<int32_t>(mapping.at(value)) };
      driver->setControl(&ctrl);
      ret = true;
      dirty = false;
    }
  }
  return ret;
}

V4l2Codec::V4l2Bitrate::V4l2Bitrate(const std::shared_ptr<V4l2Codec> & instance, bool direction)
  : Impl(instance, direction)
{
  auto driver = getDriver();
  if (driver) {
    v4l2_queryctrl ctrl = {};
    ctrl.id = V4L2_CID_MPEG_VIDEO_BITRATE;
    driver->queryControl(&ctrl);
    value = ctrl.default_value;

    v4l2_queryctrl ctrl_mode = {};
    ctrl_mode.id = V4L2_CID_MPEG_VIDEO_BITRATE_MODE;
    driver->queryControl(&ctrl_mode);

    std::for_each(bitrateModeMapping.begin(), bitrateModeMapping.end(), [&](const auto & p) {
      v4l2_querymenu querymenu = {};
      querymenu.id = V4L2_CID_MPEG_VIDEO_BITRATE_MODE;
      querymenu.index = p.second;
      if (driver->queryMenu(&querymenu) == 0) {
        supportedMode.push_back(p.first);
        if (ctrl_mode.default_value == p.second) {
          mode = p.first;
        }
      }
    });

    v4l2_queryctrl ctrl_enable = {};
    ctrl_enable.id = V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE;
    driver->queryControl(&ctrl_enable);
    enable = ctrl_enable.default_value;
  }
}

V4l2Codec::V4l2Bitrate & V4l2Codec::V4l2Bitrate::operator=(Bitrate & b)
{
  if (b.enable &&
      std::find(supportedMode.begin(), supportedMode.end(), b.mode) != supportedMode.end()) {
    value = b.value;
    mode = b.mode;
    enable = b.enable;
    dirty = true;
  } else if (b.enable != enable) {
    enable = b.enable;
    dirty = true;
  }
  return *this;
}

bool V4l2Codec::V4l2Bitrate::get()
{
  auto driver = getDriver();
  if (driver) {
    v4l2_control ctrl = { V4L2_CID_MPEG_VIDEO_BITRATE, 0 };
    if (driver->getControl(&ctrl) == 0) {
      value = ctrl.value;
      v4l2_control ctrl_enable = { V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE, 0 };
      if (driver->getControl(&ctrl_enable) == 0) {
        enable = ctrl_enable.value;
      }
      v4l2_control ctrl_mode = { V4L2_CID_MPEG_VIDEO_BITRATE_MODE, 0 };
      if (driver->getControl(&ctrl_mode) == 0) {
        auto it = std::find_if(bitrateModeMapping.begin(), bitrateModeMapping.end(),
            [&ctrl_mode](const auto & p) { return p.second == ctrl_mode.value; });
        if (it != bitrateModeMapping.end()) {
          mode = it->first;
          return true;
        }
      }
    }
  }
  return false;
}

bool V4l2Codec::V4l2Bitrate::set()
{
  LOGI("%s called", __PRETTY_FUNCTION__);
  bool ret = false;
  auto driver = getDriver();
  if (driver) {
    v4l2_control ctrl = { V4L2_CID_MPEG_VIDEO_BITRATE, value };
    if (driver->setControl(&ctrl) == 0) {
      v4l2_control ctrl_mode = { V4L2_CID_MPEG_VIDEO_BITRATE_MODE,
        static_cast<int32_t>(bitrateModeMapping.at(mode)) };
      driver->setControl(&ctrl_mode) == 0;
      v4l2_control ctrl_enable = { V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE, enable };
      driver->setControl(&ctrl_enable);
      ret = true;
      dirty = false;
    }
  }
  return ret;
}

V4l2Codec::V4l2Format::V4l2Format(const std::shared_ptr<V4l2Codec> & instance, bool direction)
  : Impl(instance, direction)
{
  auto driver = getDriver();
  std::vector<uint32_t> formats = {};
  if (driver) {
    v4l2_fmtdesc fmtdesc = {};
    fmtdesc.type = INPUT_MPLANE;
    while (driver->enumFormat(&fmtdesc) == 0) {
      formats.push_back(fmtdesc.pixelformat);
      fmtdesc.index++;
    }

    fmtdesc = {};
    fmtdesc.type = OUTPUT_MPLANE;
    while (driver->enumFormat(&fmtdesc) == 0) {
      formats.push_back(fmtdesc.pixelformat);
      fmtdesc.index++;
    }

    std::for_each(formats.begin(), formats.end(), [&](const auto & fmt) {
      auto it = std::find_if(formatMapping.begin(), formatMapping.end(),
          [&fmt](const auto & p) { return p.second == fmt; });
      if (it != formatMapping.end()) {
        supportedFormats.push_back(it->first);
      }
    });
    get();
  }
}

V4l2Codec::V4l2Format & V4l2Codec::V4l2Format::operator=(Format & f)
{
  if (std::find(supportedFormats.begin(), supportedFormats.end(), f) != supportedFormats.end()) {
    format = f;
    dirty = true;
    LOGI("%s set new format %d", __PRETTY_FUNCTION__, f);
  }
  return *this;
}

V4l2Codec::V4l2Format & V4l2Codec::V4l2Format::operator=(Resolution & f)
{
  LOGI("%s set new width %d height %d", __PRETTY_FUNCTION__, f.width, f.height);
  width = f.width;
  height = f.height;
  dirty = true;
  return *this;
}

bool V4l2Codec::V4l2Format::get()
{
  auto codec = getCodec();
  auto driver = getDriver();
  if (codec && driver) {
    auto planeType = direction ? INPUT_MPLANE : OUTPUT_MPLANE;
    v4l2_format fmt = { static_cast<uint32_t>(planeType), {} };
    if (0 > driver->getFormat(&fmt)) {
      return false;
    }

    if (memcmp(&v4l2_fmt, &fmt, sizeof(v4l2_fmt)) == 0) {
      return true;
    }

    v4l2_fmt = fmt;
    auto it = std::find_if(formatMapping.begin(), formatMapping.end(),
        [&fmt](const auto & p) { return p.second == fmt.fmt.pix.pixelformat; });
    if (it != formatMapping.end()) {
      format = it->first;
    }
    if (not dirty) {
      width = fmt.fmt.pix.width;
      height = fmt.fmt.pix.height;
    }
  }
  return true;
}

bool V4l2Codec::V4l2Format::set()
{
  LOGI("%s dir %b called", __PRETTY_FUNCTION__, direction);
  bool ret = false;
  auto codec = getCodec();
  auto driver = getDriver();
  if (codec && driver) {
    auto planeType = direction ? INPUT_MPLANE : OUTPUT_MPLANE;
    ret = driver->setCodecPixelFmt(planeType, formatMapping.at(format)) == 0;
    v4l2_format fmt = operator v4l2_format();
    if (fmt.fmt.pix_mp.width != width && dirty) {
      fmt.fmt.pix_mp.width = width;
    }
    if (fmt.fmt.pix_mp.height != height && dirty) {
      fmt.fmt.pix_mp.height = height;
    }
    ret = driver->setFormat(&fmt) == 0;
    dirty = false;
  }
  return ret;
}

V4l2Codec::V4l2Format::operator v4l2_format()
{
  get();
  return v4l2_fmt;
}

V4l2Codec::V4l2Format::operator MemoryView()
{
  get();
  MemoryView view = {};
  view.width = v4l2_fmt.fmt.pix_mp.width;
  view.height = v4l2_fmt.fmt.pix_mp.height;
  view.pixelfmt = format;
  view.type = Memory::determineType(format);
  view.num_planes = v4l2_fmt.fmt.pix_mp.num_planes;
  for (uint32_t index = 0; index < view.num_planes; index++) {
    view.planes[index].size = v4l2_fmt.fmt.pix_mp.plane_fmt[index].sizeimage;
    view.planes[index].stride = v4l2_fmt.fmt.pix_mp.plane_fmt[index].bytesperline;
    view.planes[index].scanline = v4l2_fmt.fmt.pix_mp.height;
  }

  return view;
}

V4l2Codec::V4l2Framerate::V4l2Framerate(const std::shared_ptr<V4l2Codec> & instance, bool direction)
  : Impl(instance, direction)
{
  value = 30;
  dirty = true;
}

V4l2Codec::V4l2Framerate & V4l2Codec::V4l2Framerate::operator=(Framerate & f)
{
  value = f.value;
  dirty = true;
  return *this;
}

V4l2Codec::V4l2Framerate::operator v4l2_streamparm()
{
  param.type = direction == INPUT_PORT ? INPUT_MPLANE : OUTPUT_MPLANE;
  auto & timeperframe =
      direction == INPUT_PORT ? param.parm.output.timeperframe : param.parm.output.timeperframe;
  timeperframe.numerator = 1;
  timeperframe.denominator = value;
  return param;
}

bool V4l2Codec::V4l2Framerate::set()
{
  bool ret = false;
  if (auto driver = getDriver()) {
    v4l2_streamparm reqs = this->operator v4l2_streamparm();
    if (0 == driver->setParam(&reqs)) {
      ret = true;
    }
  }
  return ret;
}

bool V4l2Codec::V4l2Framerate::get()
{
  return false;
}

V4l2Codec::V4l2BufferCount::V4l2BufferCount(const std::shared_ptr<V4l2Codec> & instance,
    bool direction)
  : Impl(instance, direction)
{
  auto count_pair = default_counts[instance->type];
  if (direction) {
    value = count_pair[0];
  } else {
    value = count_pair[1];
  }
  dirty = true;
}

V4l2Codec::V4l2BufferCount & V4l2Codec::V4l2BufferCount::operator=(BufferCount & b)
{
  value = b.value;
  dirty = true;
  return *this;
}

V4l2Codec::V4l2BufferCount::operator v4l2_requestbuffers() const
{
  v4l2_requestbuffers bufs = {};
  bufs.count = value;
  bufs.type = direction ? INPUT_MPLANE : OUTPUT_MPLANE;
  bufs.memory = V4L2_MEMORY_DMABUF;
  return bufs;
}

bool V4l2Codec::V4l2BufferCount::set()
{
  LOGI("%s called", __PRETTY_FUNCTION__);
  bool ret = false;
  if (auto driver = getDriver()) {
    v4l2_requestbuffers reqs = this->operator v4l2_requestbuffers();
    if (0 == driver->reqBufs(&reqs)) {
      value = reqs.count;
      ret = true;
    }
  }
  return ret;
}

bool V4l2Codec::V4l2BufferCount::get()
{
  return false;
}
}  // namespace qrb::video_v4l2
