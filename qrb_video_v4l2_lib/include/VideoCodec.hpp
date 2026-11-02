/*
**************************************************************************************************
* Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
**************************************************************************************************
*/

#ifndef QRB_VIDEO_V4L2__VIDEOCODEC_HPP_
#define QRB_VIDEO_V4L2__VIDEOCODEC_HPP_
#include <any>
#include <cstdint>
#include <string>

#include "Buffer.hpp"

namespace qrb::video_v4l2
{
struct Setting
{
  enum
  {
    PROFILE,
    LEVEL,
    BITRATE,
    FRAMERATE,
    RESOLUTION,
    FORMAT,
    COUNT,
  };

  uint32_t type;
  std::any data;

  template <typename T>
  static Setting create(T & setting)
  {
    Setting s = {};
    s.type = T::type;
    s.data = setting;
    return s;
  }

  static Setting create(Format & format)
  {
    Setting s = {};
    s.type = FORMAT;
    s.data = format;
    return s;
  }
};

constexpr char MIME_H264[] = "video/x-h264";
constexpr char MIME_H265[] = "video/x-h265";
enum class Format;

struct Profile
{
  enum : uint32_t
  {
    AVC_BASELINE,
    AVC_CONSTRAINED_BASELINE,
    AVC_MAIN,
    AVC_HIGH,
    AVC_CONSTRAINED_HIGH,

    HEVC_MAIN,
    HEVC_MAIN10,
    HEVC_MAIN_STILL,
  };

  uint32_t value;

  constexpr static uint32_t type = Setting::PROFILE;
};

struct Level
{
  enum : uint32_t
  {
    AVC_1_0,
    AVC_1_B,
    AVC_1_1,
    AVC_1_2,
    AVC_1_3,
    AVC_2_0,
    AVC_2_1,
    AVC_2_2,
    AVC_3_0,
    AVC_3_1,
    AVC_3_2,
    AVC_4_0,
    AVC_4_1,
    AVC_4_2,
    AVC_5_0,
    AVC_5_1,
    AVC_5_2,
    AVC_6_0,
    AVC_6_1,
    AVC_6_2,

    HEVC_1_0,
    HEVC_2_0,
    HEVC_2_1,
    HEVC_3_0,
    HEVC_3_1,
    HEVC_4_0,
    HEVC_4_1,
    HEVC_5_0,
    HEVC_5_1,
    HEVC_5_2,
    HEVC_6_0,
    HEVC_6_1,
    HEVC_6_2,
  };

  uint32_t value;

  constexpr static uint32_t type = Setting::LEVEL;
};

struct Bitrate
{
  enum Mode
  {
    OFF,
    CBR,
    VBR,
  };

  constexpr static uint32_t type = Setting::BITRATE;
  int32_t value;
  Mode mode;
  bool enable;
};

struct Framerate
{
  constexpr static uint32_t type = Setting::FRAMERATE;
  uint32_t value;
};

struct Resolution
{
  constexpr static uint32_t type = Setting::RESOLUTION;
  uint32_t width;
  uint32_t height;
};

struct BufferCount
{
  constexpr static uint32_t type = Setting::COUNT;
  size_t value;
};

enum class CodecType
{
  VideoEncoder,
  VideoDecoder,
};

class VideoCodec
{
public:
  enum class Target
  {
    V4l2,
    Gst,
  };

  enum class Error
  {
    None,
    Device,
    Negotiation,
    Memory,
    Timeout,
    Internal,
  };

  VideoCodec() = default;

  virtual ~VideoCodec() = default;

  virtual bool configure(const Setting & s) = 0;

  virtual bool start() = 0;

  virtual bool stop() = 0;

  virtual bool seek() = 0;

  virtual bool pause() = 0;

  class Notifier
  {
  public:
    virtual ~Notifier() = default;

    virtual bool onBufferAvailable(const std::shared_ptr<Buffer> & item) = 0;

    // Not pure, so existing subclasses keep compiling unchanged. Note this does add a slot
    // to Notifier's vtable: a subclass built against the pre-0.2.0 header has no such slot,
    // so the library must not dispatch onError() on one. Safe here because the library and
    // its consumers are built and shipped together; SOVERSION stays 1 on that basis.
    virtual void onError(Error code, const std::string & message) {}
  };

  virtual std::shared_ptr<Buffer> acquireBuffer() = 0;

  virtual bool queueBuffer(const std::shared_ptr<Buffer> & item) = 0;

  virtual bool dispatchBuffer(const std::shared_ptr<Buffer> & item) = 0;

  virtual void setNotifier(const std::shared_ptr<Notifier> & notifier)
  {
    this->notifier = notifier;
  }

protected:
  // Backends report failures through here rather than each re-implementing the weak_ptr
  // dance. Safe to call from any thread: Client hops onto its own looper before the
  // client-supplied Notifier is invoked.
  void notifyError(Error code, const std::string & message) const
  {
    if (const auto callback = notifier.lock()) {
      callback->onError(code, message);
    }
  }

  std::weak_ptr<Notifier> notifier;
};
}  // namespace qrb::video_v4l2

#endif  // QRB_VIDEO_V4L2__VIDEOCODEC_HPP_
