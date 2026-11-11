/*
**************************************************************************************************
* Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
**************************************************************************************************
*/

#include "GstreamerCodec.hpp"

#include <gst/allocators/gstdmabuf.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/video/video.h>

#include <algorithm>
#include <chrono>

#include "Log.hpp"

namespace qrb::video_v4l2
{
const std::unordered_map<uint32_t, const char *> GstreamerCodec::avcProfileNames = {
  { Profile::AVC_BASELINE, "baseline" },
  { Profile::AVC_CONSTRAINED_BASELINE, "constrained-baseline" },
  { Profile::AVC_MAIN, "main" },
  { Profile::AVC_HIGH, "high" },
  { Profile::AVC_CONSTRAINED_HIGH, "constrained-high" },
};

const std::unordered_map<uint32_t, const char *> GstreamerCodec::hevcProfileNames = {
  { Profile::HEVC_MAIN, "main" },
  { Profile::HEVC_MAIN10, "main-10" },
  { Profile::HEVC_MAIN_STILL, "main-still-picture" },
};

const std::unordered_map<uint32_t, const char *> GstreamerCodec::avcLevelNames = {
  { Level::AVC_1_0, "1" },
  { Level::AVC_1_B, "1b" },
  { Level::AVC_1_1, "1.1" },
  { Level::AVC_1_2, "1.2" },
  { Level::AVC_1_3, "1.3" },
  { Level::AVC_2_0, "2" },
  { Level::AVC_2_1, "2.1" },
  { Level::AVC_2_2, "2.2" },
  { Level::AVC_3_0, "3" },
  { Level::AVC_3_1, "3.1" },
  { Level::AVC_3_2, "3.2" },
  { Level::AVC_4_0, "4" },
  { Level::AVC_4_1, "4.1" },
  { Level::AVC_4_2, "4.2" },
  { Level::AVC_5_0, "5" },
  { Level::AVC_5_1, "5.1" },
  { Level::AVC_5_2, "5.2" },
  { Level::AVC_6_0, "6" },
  { Level::AVC_6_1, "6.1" },
  { Level::AVC_6_2, "6.2" },
};

const std::unordered_map<uint32_t, const char *> GstreamerCodec::hevcLevelNames = {
  { Level::HEVC_1_0, "1" },
  { Level::HEVC_2_0, "2" },
  { Level::HEVC_2_1, "2.1" },
  { Level::HEVC_3_0, "3" },
  { Level::HEVC_3_1, "3.1" },
  { Level::HEVC_4_0, "4" },
  { Level::HEVC_4_1, "4.1" },
  { Level::HEVC_5_0, "5" },
  { Level::HEVC_5_1, "5.1" },
  { Level::HEVC_5_2, "5.2" },
  { Level::HEVC_6_0, "6" },
  { Level::HEVC_6_1, "6.1" },
  { Level::HEVC_6_2, "6.2" },
};

GstreamerCodec::GstreamerCodec(CodecType type, Format compressed, const std::string & elementSuffix)
  : type(type), compressedFormat(compressed)
{
  // Guarded, and never paired with gst_deinit(): the hosting process (a ROS component
  // container) may have other GStreamer users.
  if (not gst_is_initialized()) {
    gst_init(nullptr, nullptr);
  }
  dmabuf_ = gst_dmabuf_allocator_new();

  const std::string vidc = "qvidc" + elementSuffix;
  if (auto * factory = gst_element_factory_find(vidc.c_str())) {
    gst_object_unref(factory);
    elementFamily_ = ElementFamily::Vidc;
    codecElementName_ = vidc;
  } else {
    codecElementName_ = "v4l2" + elementSuffix;
  }
}

GstreamerCodec::~GstreamerCodec()
{
  teardownPipeline();
  if (dmabuf_ != nullptr) {
    gst_object_unref(dmabuf_);
    dmabuf_ = nullptr;
  }
}

const char * GstreamerCodec::profileName(uint32_t value) const
{
  const auto & mapping = compressedFormat == Format::H264 ? avcProfileNames : hevcProfileNames;
  const auto it = mapping.find(value);
  return it == mapping.end() ? nullptr : it->second;
}

const char * GstreamerCodec::levelName(uint32_t value) const
{
  const auto & mapping = compressedFormat == Format::H264 ? avcLevelNames : hevcLevelNames;
  const auto it = mapping.find(value);
  return it == mapping.end() ? nullptr : it->second;
}

void GstreamerCodec::configureCodecElement(GstElement * codec) const {}

bool GstreamerCodec::configure(const Setting & s)
{
  switch (s.type) {
    case Setting::FORMAT:
      settings.pixelFormat = std::any_cast<Format>(s.data);
      break;
    case Setting::RESOLUTION:
      settings.resolution = std::any_cast<Resolution>(s.data);
      break;
    case Setting::FRAMERATE:
      settings.framerate = std::any_cast<Framerate>(s.data);
      break;
    case Setting::PROFILE:
      settings.profile = std::any_cast<Profile>(s.data);
      break;
    case Setting::LEVEL:
      settings.level = std::any_cast<Level>(s.data);
      break;
    case Setting::BITRATE:
      settings.bitrate = std::any_cast<Bitrate>(s.data);
      break;
    case Setting::COUNT:
      settings.bufferCount = std::any_cast<BufferCount>(s.data);
      break;
    default:
      LOGE("%s: unsupported setting type %d", __PRETTY_FUNCTION__, s.type);
      return false;
  }
  LOGI("%s: stored setting type %d", __PRETTY_FUNCTION__, s.type);
  return true;
}

bool GstreamerCodec::buildPipeline()
{
  pipeline_ = gst_pipeline_new("qrb-video");
  source_ = gst_element_factory_make("appsrc", "src");
  const std::string parserName = parserElementName();
  GstElement * parser =
      parserName.empty() ? nullptr : gst_element_factory_make(parserName.c_str(), "parser");
  codec_ = gst_element_factory_make(codecElementName().c_str(), "codec");
  auto * srcFilter = gst_element_factory_make("capsfilter", "src-caps");
  sink_ = gst_element_factory_make("appsink", "sink");

  if (pipeline_ == nullptr || source_ == nullptr ||
      (parser == nullptr and not parserName.empty()) || codec_ == nullptr || srcFilter == nullptr ||
      sink_ == nullptr) {
    LOGE("%s: failed to create elements (is %s available?)", __PRETTY_FUNCTION__,
        codecElementName().c_str());
    reportError(Error::Internal, "failed to create GStreamer elements for " + codecElementName());
    // Unref whatever did get created; unparented elements have to be dropped by hand.
    for (auto ** e : { &source_, &parser, &codec_, &srcFilter, &sink_ }) {
      if (*e != nullptr) {
        gst_object_unref(*e);
        *e = nullptr;
      }
    }
    if (pipeline_ != nullptr) {
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
    }
    return false;
  }

  // We supply PTS ourselves from the producer's timestamps, and must not be clock-slaved:
  // sync=true on the sink would add latency and drop frames under jitter.
  g_object_set(source_, "is-live", TRUE, "format", GST_FORMAT_TIME, "do-timestamp", FALSE,
      "emit-signals", FALSE, nullptr);

  if (auto * caps = buildSinkCaps()) {
    g_object_set(source_, "caps", caps, nullptr);
    gst_caps_unref(caps);
  }
  if (auto * caps = buildSrcCaps()) {
    g_object_set(srcFilter, "caps", caps, nullptr);
    gst_caps_unref(caps);
  }

  configureCodecElement(codec_);

  g_object_set(sink_, "sync", FALSE, "emit-signals", TRUE, "drop", FALSE, nullptr);
  if (settings.bufferCount.has_value()) {
    g_object_set(sink_, "max-buffers", static_cast<guint>(settings.bufferCount->value), nullptr);
  }
  g_signal_connect(sink_, "new-sample", G_CALLBACK(onNewSample), this);

  if (auto * sinkPad = gst_element_get_static_pad(sink_, "sink")) {
    gst_pad_add_probe(
        sinkPad, GST_PAD_PROBE_TYPE_QUERY_DOWNSTREAM,
        [](GstPad *, GstPadProbeInfo * info, gpointer) {
          if (auto * query = GST_PAD_PROBE_INFO_QUERY(info);
              GST_QUERY_TYPE(query) == GST_QUERY_ALLOCATION) {
            gst_query_add_allocation_meta(query, GST_VIDEO_META_API_TYPE, nullptr);
          }
          return GST_PAD_PROBE_OK;
        },
        nullptr, nullptr);
    gst_object_unref(sinkPad);
  }

  gst_bin_add_many(GST_BIN(pipeline_), source_, codec_, srcFilter, sink_, nullptr);
  bool linked = false;
  if (parser != nullptr) {
    gst_bin_add(GST_BIN(pipeline_), parser);
    linked = gst_element_link_many(source_, parser, codec_, srcFilter, sink_, nullptr);
  } else {
    linked = gst_element_link_many(source_, codec_, srcFilter, sink_, nullptr);
  }
  if (not linked) {
    LOGE("%s: failed to link pipeline", __PRETTY_FUNCTION__);
    reportError(Error::Negotiation, "failed to link GStreamer pipeline");
    teardownPipeline();
    return false;
  }

  return true;
}

void GstreamerCodec::teardownPipeline()
{
  if (pipeline_ == nullptr) {
    return;
  }
  gst_element_set_state(pipeline_, GST_STATE_NULL);
  // Children are owned by the bin and go with it.
  gst_object_unref(pipeline_);
  pipeline_ = nullptr;
  source_ = nullptr;
  codec_ = nullptr;
  sink_ = nullptr;
}

bool GstreamerCodec::start()
{
  if (state_ != STOPPED) {
    LOGE("%s: called with invalid state %d", __PRETTY_FUNCTION__, state_.load());
    return false;
  }

  {
    std::lock_guard lock(drainMutex_);
    eosReceived_ = false;
    errored_ = false;
  }

  if (not buildPipeline()) {
    return false;
  }

  busRunning_ = true;
  busThread_ = std::thread([this]() { busLoop(); });

  const auto ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    LOGE("%s: failed to set pipeline to PLAYING", __PRETTY_FUNCTION__);
    reportError(Error::Device, "failed to start GStreamer pipeline");
    busRunning_ = false;
    if (busThread_.joinable()) {
      busThread_.join();
    }
    teardownPipeline();
    return false;
  }

  state_ = STARTED;
  LOGI("%s: %s pipeline started", __PRETTY_FUNCTION__, codecElementName().c_str());
  return true;
}

bool GstreamerCodec::stop()
{
  if (state_ != STARTED) {
    return true;
  }
  state_ = STOPPED;

  // Draining is just EOS here: appsrc propagates it, the codec element issues
  // V4L2_ENC_CMD_STOP / V4L2_DEC_CMD_STOP internally, and the bus tells us when the last
  // frame is out.
  bool drained = false;
  if (source_ != nullptr) {
    if (gst_app_src_end_of_stream(GST_APP_SRC(source_)) == GST_FLOW_OK) {
      drained = waitForEos(kDrainTimeoutMs);
      if (not drained) {
        LOGE("%s: timed out waiting for EOS", __PRETTY_FUNCTION__);
      }
    } else {
      LOGE("%s: failed to signal end-of-stream", __PRETTY_FUNCTION__);
    }
  }

  busRunning_ = false;
  if (busThread_.joinable()) {
    busThread_.join();
  }
  teardownPipeline();

  {
    std::lock_guard lock(poolMutex_);
    inputPool_.reset();
  }

  LOGI("%s: pipeline stopped (drained %d)", __PRETTY_FUNCTION__, drained);
  return drained;
}

bool GstreamerCodec::seek()
{
  return false;
}

bool GstreamerCodec::pause()
{
  return false;
}

bool GstreamerCodec::waitForEos(int timeoutMs)
{
  std::unique_lock lock(drainMutex_);
  const bool ok = drainCv_.wait_for(
      lock, std::chrono::milliseconds(timeoutMs), [this]() { return eosReceived_ || errored_; });
  return ok && eosReceived_ && not errored_;
}

void GstreamerCodec::busLoop()
{
  auto * bus = gst_element_get_bus(pipeline_);
  if (bus == nullptr) {
    LOGE("%s: pipeline has no bus", __PRETTY_FUNCTION__);
    return;
  }

  while (busRunning_) {
    // Bounded pop rather than a GMainLoop watch, so this works inside a ROS component
    // container that has no GLib main loop running.
    auto * msg = gst_bus_timed_pop_filtered(bus, 100 * GST_MSECOND,
        static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_WARNING | GST_MESSAGE_EOS));
    if (msg == nullptr) {
      continue;
    }

    switch (GST_MESSAGE_TYPE(msg)) {
      case GST_MESSAGE_ERROR: {
        GError * err = nullptr;
        gchar * debug = nullptr;
        gst_message_parse_error(msg, &err, &debug);
        const std::string text = err != nullptr ? err->message : "unknown GStreamer error";
        LOGE("%s: bus error from %s: %s (%s)", __PRETTY_FUNCTION__, GST_OBJECT_NAME(msg->src),
            text.c_str(), debug != nullptr ? debug : "");
        {
          std::lock_guard lock(drainMutex_);
          errored_ = true;
        }
        drainCv_.notify_all();
        // A failure to negotiate is the expected shape of "the driver cannot do what was
        // asked", so keep it distinguishable from a device-level fault.
        const bool negotiation = err != nullptr && err->domain == GST_STREAM_ERROR &&
                                 err->code == GST_STREAM_ERROR_FORMAT;
        reportError(negotiation ? Error::Negotiation : Error::Device, text);
        if (err != nullptr) {
          g_error_free(err);
        }
        g_free(debug);
      } break;
      case GST_MESSAGE_WARNING: {
        GError * err = nullptr;
        gchar * debug = nullptr;
        gst_message_parse_warning(msg, &err, &debug);
        LOGE("%s: bus warning from %s: %s", __PRETTY_FUNCTION__, GST_OBJECT_NAME(msg->src),
            err != nullptr ? err->message : "unknown");
        if (err != nullptr) {
          g_error_free(err);
        }
        g_free(debug);
      } break;
      case GST_MESSAGE_EOS: {
        LOGI("%s: EOS reached", __PRETTY_FUNCTION__);
        {
          std::lock_guard lock(drainMutex_);
          eosReceived_ = true;
        }
        drainCv_.notify_all();
      } break;
      default:
        break;
    }
    gst_message_unref(msg);
  }

  gst_object_unref(bus);
}

void GstreamerCodec::reportError(Error code, const std::string & message)
{
  notifyError(code, message);
}

size_t GstreamerCodec::compressedBufferSize() const
{
  // The V4L2 path learns this from the driver's sizeimage after S_FMT; there is no
  // equivalent here before the pipeline runs, so reserve enough for a worst-case access
  // unit. One byte per pixel is generous for any real bitstream.
  const size_t pixels = static_cast<size_t>(settings.resolution.width) * settings.resolution.height;
  constexpr size_t kMinimum = 1u << 20;  // 1 MiB
  constexpr size_t kUnknown = 4u << 20;  // 4 MiB, used when resolution was never set
  if (pixels == 0) {
    return kUnknown;
  }
  return std::max(pixels, kMinimum);
}

std::shared_ptr<Buffer> GstreamerCodec::acquireBuffer()
{
  std::unique_lock lock(poolMutex_);
  if (not inputPool_) {
    // Pool rather than allocate-per-call: the CompressedImage decode path asks for a buffer
    // every frame.
    auto view =
        MemoryView::create(compressedFormat, settings.resolution.width, settings.resolution.height);
    view.num_planes = 1;
    view.planes[0].size = compressedBufferSize();

    auto allocator = std::make_shared<DmabufAllocator>();
    allocator->set(view);
    inputPool_ = std::make_shared<BufferPool>();
    inputPool_->registerAllocator(allocator);
  }
  auto pool = inputPool_;
  lock.unlock();

  std::shared_ptr<Buffer> buffer;
  pool->acquire(buffer);
  if (not buffer) {
    LOGE("%s: input pool exhausted", __PRETTY_FUNCTION__);
  }
  return buffer;
}

bool GstreamerCodec::queueBuffer(const std::shared_ptr<Buffer> & item)
{
  if (state_ != STARTED || source_ == nullptr) {
    LOGE("%s: codec is not started", __PRETTY_FUNCTION__);
    return false;
  }

  if (item->isEOS()) {
    LOGI("%s: draining on EOS input", __PRETTY_FUNCTION__);
    return gst_app_src_end_of_stream(GST_APP_SRC(source_)) == GST_FLOW_OK;
  }

  // The encoder's input is raw video and needs a plane layout; the decoder's is an opaque
  // access unit. Build the view from our own settings rather than from the incoming
  // Allocation, because producers leave stride/scanline unset (the ROS nodes report
  // num_planes = 1 with no strides) and determinePlanes() is what encodes the required
  // alignment.
  const bool raw = type == CodecType::VideoEncoder;
  MemoryView view = raw ? MemoryView::create(settings.pixelFormat, settings.resolution.width,
                              settings.resolution.height) :
                          item->view();

  auto * gstbuf = wrapBuffer(dmabuf_, item, view, raw);
  if (gstbuf == nullptr) {
    reportError(Error::Memory, "failed to wrap input buffer as a dmabuf GstBuffer");
    return false;
  }

  // Takes ownership of gstbuf regardless of outcome.
  const auto flow = gst_app_src_push_buffer(GST_APP_SRC(source_), gstbuf);
  if (flow != GST_FLOW_OK) {
    LOGE("%s: push failed with %s", __PRETTY_FUNCTION__, gst_flow_get_name(flow));
    return false;
  }
  return true;
}

bool GstreamerCodec::dispatchBuffer(const std::shared_ptr<Buffer> & item)
{
  const auto callback = notifier.lock();
  if (not callback) {
    return false;
  }
  return callback->onBufferAvailable(item);
}

GstFlowReturn GstreamerCodec::onNewSample(GstAppSink * sink, gpointer user_data)
{
  return static_cast<GstreamerCodec *>(user_data)->handleNewSample(sink);
}

GstFlowReturn GstreamerCodec::handleNewSample(GstAppSink * sink)
{
  auto * sample = gst_app_sink_pull_sample(sink);
  if (sample == nullptr) {
    return GST_FLOW_EOS;
  }

  auto buffer = wrapSample(sample, outputFormat(), sequence_++);
  // wrapSample took its own ref if it succeeded.
  gst_sample_unref(sample);

  if (not buffer) {
    reportError(Error::Memory, "codec output was not a dmabuf");
    return GST_FLOW_ERROR;
  }

  // Runs on the GStreamer streaming thread. dispatchBuffer() hands off to the Client's
  // looper, so publishing never blocks this thread.
  dispatchBuffer(buffer);
  return GST_FLOW_OK;
}

bool GstreamerCodec::onAcquireBuffer(std::shared_ptr<Buffer> & item)
{
  item = acquireBuffer();
  return item != nullptr;
}

bool GstreamerCodec::onQueueBuffer(const std::shared_ptr<Buffer> & item)
{
  return queueBuffer(item);
}

bool GstreamerCodec::onConfigure(const Setting & s)
{
  return configure(s);
}

bool GstreamerCodec::onStart()
{
  return start();
}

bool GstreamerCodec::onStop()
{
  return stop();
}

bool GstreamerCodec::onPause()
{
  return pause();
}

bool GstreamerCodec::onSeek()
{
  return seek();
}
}  // namespace qrb::video_v4l2
