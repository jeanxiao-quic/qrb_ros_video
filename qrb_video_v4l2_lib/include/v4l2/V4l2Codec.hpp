/*
 **************************************************************************************************
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 **************************************************************************************************
 */

#ifndef QRB_VIDEO_V4L2__V4L2CODEC_HPP_
#define QRB_VIDEO_V4L2__V4L2CODEC_HPP_

#include <array>
#include <linux/v4l2_vidc_extensions.hpp>
#include <memory>
#include <string>
#include <unordered_map>

#include "BufferChannel.hpp"
#include "BufferPool.hpp"
#include "V4l2Buffer.hpp"
#include "V4l2Driver.hpp"
#include "V4l2Memory.hpp"
#include "VideoCodec.hpp"

namespace qrb::video_v4l2
{
class V4l2Codec;

class EventHandler : public Handler
{
public:
  EventHandler(const std::shared_ptr<V4l2Codec> & c, Handler * another)
    : Handler(another), self(c){};

  bool handleMessage(const std::shared_ptr<Message> & msg) override;

private:
  std::shared_ptr<V4l2Codec> self;
};

class V4l2Codec : public VideoCodec,
                  public ChannelCB,
                  public V4l2Driver::Callback,
                  public std::enable_shared_from_this<V4l2Codec>
{
public:
  explicit V4l2Codec(const CodecType type, std::string name);

  ~V4l2Codec() override = default;

  bool start() override;

  bool stop() override;

  bool configure(const Setting & s) override;

  bool seek() override;

  bool pause() override;

  bool dispatchBuffer(const std::shared_ptr<Buffer> & item) override;

  std::shared_ptr<Buffer> acquireBuffer() override;

  bool queueBuffer(const std::shared_ptr<Buffer> & item) override;

  bool onAcquireBuffer(std::shared_ptr<Buffer> & item) override;

  bool onQueueBuffer(const std::shared_ptr<Buffer> & item) override;

  bool onConfigure(const Setting & s) override;

  bool onStart() override;

  bool onStop() override;

  bool onPause() override;

  bool onSeek() override;

  bool onV4l2BufferDone(v4l2_buffer * buffer) override;

  bool onV4l2EventDone(v4l2_event * event) override;

  bool onV4l2Error(error_t error) override;

protected:
  static const std::unordered_map<uint32_t, int32_t> avcProfileMapping;
  static const std::unordered_map<uint32_t, int32_t> hevcProfileMapping;
  static const std::unordered_map<uint32_t, int32_t> avcLevelMapping;
  static const std::unordered_map<uint32_t, int32_t> hevcLevelMapping;
  static const std::unordered_map<Bitrate::Mode, int32_t> bitrateModeMapping;
  static const std::unordered_map<Format, uint32_t> formatMapping;

  CodecType type;
  Format compressedFormat;
  V4l2Memory::Type memoryType;

  enum State
  {
    STOPPED,
    STARTED,
    PAUSED,
    FLUSHING,
    RECONFIGURING,
  };

  enum
  {
    MSG_FLUSH_PORT,
    MSG_RECONFIGURE_PORT,
    MSG_FEED_OUTPUT_BUFFER,
  };

  std::atomic<State> state;

  std::shared_ptr<V4l2Driver> driver;
  std::shared_ptr<BufferPool> inputPool;
  std::shared_ptr<BufferPool> outputPool;
  std::array<std::map<uint32_t, std::shared_ptr<V4l2Buffer> >, 2> buffer_queued_;
  std::mutex mutex_;
  std::promise<bool> emptied_;
  std::shared_ptr<EventHandler> handler_;

  constexpr static bool INPUT_PORT = true;
  constexpr static bool OUTPUT_PORT = false;

  void init();

  void populateSettings() const;

  std::shared_ptr<Buffer> acquireInputBuffer() const;

  std::shared_ptr<Buffer> acquireOutputBuffer() const;

  bool queueInputBuffer(const std::shared_ptr<Buffer> & item);

  bool queueOutputBuffer(const std::shared_ptr<Buffer> & item);

  bool feedOutputBuffer();

  bool flush(bool input);

  bool prepareForDispatch(std::shared_ptr<V4l2Buffer> & buf, v4l2_buffer * vb);

  virtual bool drain();

  virtual bool reconfigurePort(bool port);

  virtual bool startStreaming(bool port);

  virtual bool stopStreaming(bool port);

  virtual bool startPort(bool input);

  friend class EventHandler;

  template <typename T>
  std::shared_ptr<T> get(bool direction)
  {
    auto it = settings.find(T::type);
    if (it == settings.end()) {
      auto input_setting = std::make_shared<T>(shared_from_this(), true);
      auto output_setting = std::make_shared<T>(shared_from_this(), false);
      settings[T::type] = std::make_pair(input_setting, output_setting);
      return direction ? input_setting : output_setting;
    }
    auto pair = it->second;
    return std::dynamic_pointer_cast<T>(direction ? pair.first : pair.second);
  }

  std::shared_ptr<V4l2Driver> getDriver() const { return driver; }

  struct Impl;

  std::map<uint32_t, std::pair<std::shared_ptr<Impl>, std::shared_ptr<Impl> > > settings;

  template <typename AllocatorType>
  void prepareBufferPool(bool input);

  struct Impl
  {
    Impl() = default;

    explicit Impl(const std::shared_ptr<V4l2Codec> & instance, bool input)
      : instance(instance), direction(input)
    {
    }

    virtual ~Impl() = default;

    virtual bool get() = 0;

    virtual bool set() = 0;

    std::weak_ptr<V4l2Codec> instance;
    bool direction;
    bool dirty = false;

    std::shared_ptr<V4l2Driver> getDriver() const
    {
      if (auto codec = instance.lock()) {
        return codec->driver;
      }
      return nullptr;
    }

    std::shared_ptr<V4l2Codec> getCodec() const
    {
      if (auto codec = instance.lock()) {
        return codec;
      }
      return nullptr;
    }
  };

  struct V4l2Profile : Profile, Impl
  {
    V4l2Profile() = default;

    explicit V4l2Profile(const std::shared_ptr<V4l2Codec> & instance, bool direction);

    V4l2Profile & operator=(Profile & p);

    bool get() override;

    bool set() override;

    std::vector<decltype(value)> supportedProfile;
  };

  struct V4l2Level : Level, Impl
  {
    V4l2Level() = default;

    explicit V4l2Level(const std::shared_ptr<V4l2Codec> & instance, bool direction);

    V4l2Level & operator=(Level & l);

    explicit operator uint32_t() const;

    bool get() override;

    bool set() override;

    std::vector<decltype(value)> supportedLevel;
  };

  struct V4l2Bitrate : Bitrate, Impl
  {
    V4l2Bitrate() = default;

    explicit V4l2Bitrate(const std::shared_ptr<V4l2Codec> & instance, bool direction);

    V4l2Bitrate & operator=(Bitrate & b);

    explicit operator uint32_t() const;

    explicit operator v4l2_mpeg_video_bitrate_mode() const;

    bool get() override;

    bool set() override;

    std::vector<decltype(mode)> supportedMode;
  };

  struct V4l2Format : Resolution, Impl
  {
    V4l2Format() = default;

    explicit V4l2Format(const std::shared_ptr<V4l2Codec> & instance, bool direction);

    V4l2Format & operator=(Format & f);

    V4l2Format & operator=(Resolution & f);

    explicit operator v4l2_format();

    explicit operator MemoryView();

    Format format;
    v4l2_format v4l2_fmt;
    std::vector<Format> supportedFormats;

    constexpr static uint32_t type = Setting::FORMAT;

    bool get() override;

    bool set() override;
  };

  struct V4l2Framerate : Framerate, Impl
  {
    V4l2Framerate() = default;

    explicit V4l2Framerate(const std::shared_ptr<V4l2Codec> & driver, bool direction);

    V4l2Framerate & operator=(Framerate & f);

    explicit operator v4l2_streamparm();

    bool set() override;

    bool get() override;

    v4l2_streamparm param;
  };

  struct V4l2BufferCount : BufferCount, Impl
  {
    V4l2BufferCount() = default;

    explicit V4l2BufferCount(const std::shared_ptr<V4l2Codec> & instance, bool direction);

    V4l2BufferCount & operator=(BufferCount & b);

    explicit operator v4l2_requestbuffers() const;

    std::map<CodecType, std::array<int32_t, 2> > default_counts = {
      { CodecType::VideoDecoder, { 4, 12 } },
      { CodecType::VideoEncoder, { 12, 4 } },
    };

    bool get() override;

    bool set() override;
  };
};
}  // namespace qrb::video_v4l2

#endif  // QRB_VIDEO_V4L2__V4L2CODEC_HPP_
