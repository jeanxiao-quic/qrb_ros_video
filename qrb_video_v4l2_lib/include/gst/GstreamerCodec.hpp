/*
 **************************************************************************************************
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 **************************************************************************************************
 */

#ifndef QRB_VIDEO_V4L2__GSTREAMERCODEC_HPP_
#define QRB_VIDEO_V4L2__GSTREAMERCODEC_HPP_

#include <gst/app/gstappsink.h>
#include <gst/gst.h>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>

#include "BufferChannel.hpp"
#include "BufferPool.hpp"
#include "DmabufAllocator.hpp"
#include "GstreamerMemory.hpp"
#include "VideoCodec.hpp"

namespace qrb::video_v4l2
{
// A codec backed by the gstv4l2 elements (v4l2h264enc / v4l2h265enc / v4l2h264dec / ...)
// rather than by direct ioctls. Same device, same driver -- but drain/EOS, caps
// renegotiation and buffer recycling are owned upstream instead of by this library.
//
// Base-class set matches V4l2Codec so BufferChannel/Client/Looper/Handler and the threading
// model are reused unchanged.
class GstreamerCodec : public VideoCodec,
                       public ChannelCB,
                       public std::enable_shared_from_this<GstreamerCodec>
{
public:
  GstreamerCodec(CodecType type, Format compressed);

  ~GstreamerCodec() override;

  bool configure(const Setting & s) override;

  bool start() override;

  bool stop() override;

  bool seek() override;

  bool pause() override;

  std::shared_ptr<Buffer> acquireBuffer() override;

  bool queueBuffer(const std::shared_ptr<Buffer> & item) override;

  bool dispatchBuffer(const std::shared_ptr<Buffer> & item) override;

  bool onAcquireBuffer(std::shared_ptr<Buffer> & item) override;

  bool onQueueBuffer(const std::shared_ptr<Buffer> & item) override;

  bool onConfigure(const Setting & s) override;

  bool onStart() override;

  bool onStop() override;

  bool onPause() override;

  bool onSeek() override;

protected:
  // Everything configure() collects, applied when the pipeline is built. Caps and element
  // properties must be set before the pipeline reaches PLAYING, so nothing is pushed to
  // GStreamer until start().
  struct Settings
  {
    Format pixelFormat = Format::NV12;
    Resolution resolution = { 0, 0 };
    Framerate framerate = { 30 };
    std::optional<Profile> profile;
    std::optional<Level> level;
    std::optional<Bitrate> bitrate;
    // Setting::COUNT carries no direction, so it maps to the consumer-visible queue depth
    // only. The input side uses BufferPool's own default capacity.
    std::optional<BufferCount> bufferCount;
  };

  // Subclass hooks. Called once, from start(), before the pipeline goes to PLAYING.
  virtual std::string codecElementName() const = 0;
  virtual std::string parserElementName() const { return {}; }
  virtual GstCaps * buildSinkCaps() const = 0;
  virtual GstCaps * buildSrcCaps() const = 0;
  // Format the output Buffers should report when caps give nothing better.
  virtual Format outputFormat() const = 0;
  // Applied to the codec element after it is created, e.g. io-modes and extra-controls.
  virtual void configureCodecElement(GstElement * codec) const;

  static const std::unordered_map<uint32_t, const char *> avcProfileNames;
  static const std::unordered_map<uint32_t, const char *> hevcProfileNames;
  static const std::unordered_map<uint32_t, const char *> avcLevelNames;
  static const std::unordered_map<uint32_t, const char *> hevcLevelNames;

  // Looks up a profile/level name for the compressed format in use, or nullptr.
  const char * profileName(uint32_t value) const;
  const char * levelName(uint32_t value) const;

  CodecType type;
  Format compressedFormat;
  Settings settings;

  GstElement * pipeline_ = nullptr;
  GstElement * source_ = nullptr;
  GstElement * codec_ = nullptr;
  GstElement * sink_ = nullptr;
  GstAllocator * dmabuf_ = nullptr;

  // Backs acquireBuffer(). Only the compressed-input path (CompressedImage into the
  // decoder) uses it; pooling matters because that path asks for a buffer per frame and a
  // dma_heap allocation per frame would be wasteful.
  std::shared_ptr<BufferPool> inputPool_;
  std::mutex poolMutex_;

  // Size to reserve for a pooled compressed input buffer.
  size_t compressedBufferSize() const;

private:
  enum State
  {
    STOPPED,
    STARTED,
  };

  // How long stop() waits for EOS to come back up the bus before giving up and tearing the
  // pipeline down anyway.
  static constexpr int kDrainTimeoutMs = 3000;

  std::atomic<State> state_ = STOPPED;
  std::atomic<size_t> sequence_ = 0;

  // Bus polling runs on its own thread rather than a GMainLoop: a ROS component container
  // has no GLib main loop to attach a watch to. Mirrors V4l2Driver::threadLoop().
  std::thread busThread_;
  std::atomic<bool> busRunning_ = false;

  std::mutex drainMutex_;
  std::condition_variable drainCv_;
  bool eosReceived_ = false;
  bool errored_ = false;

  bool buildPipeline();
  void teardownPipeline();
  void busLoop();
  bool waitForEos(int timeoutMs);
  void reportError(Error code, const std::string & message);

  static GstFlowReturn onNewSample(GstAppSink * sink, gpointer user_data);

  GstFlowReturn handleNewSample(GstAppSink * sink);
};
}  // namespace qrb::video_v4l2

#endif  // QRB_VIDEO_V4L2__GSTREAMERCODEC_HPP_
