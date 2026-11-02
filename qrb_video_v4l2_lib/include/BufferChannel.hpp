/*
 **************************************************************************************************
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 **************************************************************************************************
 */

#ifndef QRB_VIDEO_V4L2__BUFFERCHANNEL_HPP_
#define QRB_VIDEO_V4L2__BUFFERCHANNEL_HPP_

#include <memory>

#include "Buffer.hpp"
#include "VideoCodec.hpp"
#include "utils/Handler.hpp"
#include "utils/Looper.hpp"

namespace qrb::video_v4l2
{
class ChannelCB
{
public:
  ChannelCB() = default;
  virtual ~ChannelCB() = default;

  virtual bool onAcquireBuffer(std::shared_ptr<Buffer> & item) = 0;

  virtual bool onQueueBuffer(const std::shared_ptr<Buffer> & item) = 0;

  virtual bool onConfigure(const Setting & s) = 0;

  virtual bool onStart() = 0;

  virtual bool onStop() = 0;

  virtual bool onPause() = 0;

  virtual bool onSeek() = 0;
};

class BufferChannel : public VideoCodec, public Handler
{
public:
  // Payload for MSG_ERROR, carried through Message::data.
  struct ErrorInfo
  {
    Error code;
    std::string message;
  };

  enum
  {
    MSG_DISPATCH_BUFFER,
    MSG_ACQUIRE_BUFFER,
    MSG_QUEUE_BUFFER,
    MSG_CONFIGURE,

    MSG_START,
    MSG_STOP,
    MSG_PAUSE,
    MSG_SEEK,
    MSG_ERROR,
  };

  BufferChannel() = default;
  ~BufferChannel() override = default;

  void setChannelCB(const std::shared_ptr<ChannelCB> & cb) { this->cb = cb; }

  std::shared_ptr<Buffer> acquireBuffer() override;

  bool queueBuffer(const std::shared_ptr<Buffer> & item) override;

  bool dispatchBuffer(const std::shared_ptr<Buffer> & item) override;

  bool configure(const Setting & s) override;

  bool start() override;

  bool stop() override;

  bool seek() override;

  bool pause() override;

  bool notifyBufferAvailable(const std::shared_ptr<Buffer> & item) const
  {
    bool ret = false;
    const auto callback = notifier.lock();
    if (callback) {
      ret = callback->onBufferAvailable(item);
    }
    return ret;
  }

private:
  std::shared_ptr<ChannelCB> cb;
  std::string name;

  bool handleMessage(const std::shared_ptr<Message> & msg) override;
};

class Client : public BufferChannel, public VideoCodec::Notifier
{
public:
  bool onBufferAvailable(const std::shared_ptr<Buffer> & item) override;

  void onError(Error code, const std::string & message) override;
};
}  // namespace qrb::video_v4l2

#endif  // QRB_VIDEO_V4L2__BUFFERCHANNEL_HPP_
