/*
**************************************************************************************************
* Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
**************************************************************************************************
*/

#include "BufferChannel.hpp"

#include <cassert>

#include "Log.hpp"

namespace qrb::video_v4l2
{
bool BufferChannel::queueBuffer(const std::shared_ptr<Buffer> & item)
{
  auto msg = this->obtainMessage(MSG_QUEUE_BUFFER);
  msg->data = item;
  return this->sendMessage(msg);
}

std::shared_ptr<Buffer> BufferChannel::acquireBuffer()
{
  // Bypass the Looper: BufferPool::acquire() is mutex-protected and safe to call
  // from any thread. Going through sendMessage() serializes with other Looper work
  // (queueBuffer, flush, reconfigure) and can time out when the Looper is busy
  // processing a long V4L2 ioctl, causing spurious null returns.
  std::shared_ptr<Buffer> buffer;
  if (cb) {
    cb->onAcquireBuffer(buffer);
  }
  return buffer;
}

bool BufferChannel::dispatchBuffer(const std::shared_ptr<Buffer> & item)
{
  auto msg = this->obtainMessage(MSG_DISPATCH_BUFFER);
  msg->data = item;
  return this->sendMessageAsync(msg);
}

bool BufferChannel::configure(const Setting & s)
{
  auto msg = this->obtainMessage(MSG_CONFIGURE);
  msg->data = s;
  this->sendMessage(msg);
  return true;
}

bool BufferChannel::start()
{
  auto msg = this->obtainMessage(MSG_START);
  this->sendMessage(msg);
  return true;
}

bool BufferChannel::stop()
{
  auto msg = this->obtainMessage(MSG_STOP);
  this->sendMessage(msg);
  return true;
}

bool BufferChannel::seek()
{
  auto msg = this->obtainMessage(MSG_SEEK);
  this->sendMessage(msg);
  return true;
}

bool BufferChannel::pause()
{
  auto msg = this->obtainMessage(MSG_PAUSE);
  this->sendMessage(msg);
  return true;
}

bool BufferChannel::handleMessage(const std::shared_ptr<Message> & msg)
{
  bool ret = false;
  auto callback = cb;
  assert(callback != nullptr);
  LOGI("%s: receive msg type %x", __func__, msg->what);
  switch (msg->what) {
    case MSG_DISPATCH_BUFFER:
      ret = notifyBufferAvailable(std::any_cast<std::shared_ptr<Buffer> >(msg->data));
      break;
    case MSG_ACQUIRE_BUFFER:
      if (callback) {
        auto buffer = std::any_cast<std::shared_ptr<Buffer> >(msg->data);
        ret = callback->onAcquireBuffer(buffer);
        msg->data = buffer;
      }
      break;
    case MSG_QUEUE_BUFFER:
      if (callback) {
        ret = callback->onQueueBuffer(std::any_cast<std::shared_ptr<Buffer> >(msg->data));
      }
      break;
    case MSG_CONFIGURE:
      if (callback) {
        ret = callback->onConfigure(std::any_cast<Setting>(msg->data));
      }
      break;
    case MSG_START:
      if (callback) {
        ret = callback->onStart();
      }
      break;
    case MSG_STOP:
      if (callback) {
        ret = callback->onStop();
      }
      break;
    case MSG_PAUSE:
      if (callback) {
        ret = callback->onPause();
      }
      break;
    case MSG_SEEK:
      if (callback) {
        ret = callback->onSeek();
      }
      break;
    default:
      break;
  }

  return finishMessage(msg, ret);
}

bool Client::onBufferAvailable(const std::shared_ptr<Buffer> & item)
{
  auto ret = dispatchBuffer(item);
  return ret;
}
}  // namespace qrb::video_v4l2
