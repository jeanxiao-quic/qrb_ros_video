/*
**************************************************************************************************
* Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
**************************************************************************************************
*/

#include "GstreamerMemory.hpp"

#include <fcntl.h>
#include <gst/allocators/gstdmabuf.h>
#include <unistd.h>

#include "Log.hpp"

namespace qrb::video_v4l2
{
GstVideoFormat toGstVideoFormat(Format format)
{
  switch (format) {
    case Format::NV12:
      return GST_VIDEO_FORMAT_NV12;
    case Format::P010:
      return GST_VIDEO_FORMAT_P010_10LE;
    case Format::NV12C:
      return gst_video_format_from_string("NV12_Q08C");
    case Format::TP10C:
      return gst_video_format_from_string("NV12_Q10LE32C");
    default:
      return GST_VIDEO_FORMAT_UNKNOWN;
  }
}

const char * toGstMediaType(Format format)
{
  switch (format) {
    case Format::H264:
      return "video/x-h264";
    case Format::HEVC:
      return "video/x-h265";
    default:
      return nullptr;
  }
}

std::shared_ptr<Buffer> wrapSample(GstSample * sample, Format fallbackFormat, size_t index)
{
  auto * gstbuf = gst_sample_get_buffer(sample);
  if (gstbuf == nullptr || gst_buffer_n_memory(gstbuf) == 0) {
    LOGE("%s: sample carries no memory", __func__);
    return nullptr;
  }

  auto * mem = gst_buffer_peek_memory(gstbuf, 0);
  const bool isDmabuf = gst_is_dmabuf_memory(mem);
  // Some elements (qvidc's encoders) always hand back the bitstream in system memory. That is
  // acceptable for compressed output only: the ROS layer reads it through data()/bytesused and
  // has no need of an fd. Raw output is deliberately not accepted this way -- it would hide a
  // misconfigured pipeline, and raw consumers do need a dmabuf.
  const bool isCompressed = toGstMediaType(fallbackFormat) != nullptr;
  if (not isDmabuf and not isCompressed) {
    LOGE("%s: raw output memory is not a dmabuf", __func__);
    return nullptr;
  }

  auto allocation = std::make_shared<Allocation>();
  gsize offset = 0;
  gsize maxsize = 0;
  if (isDmabuf) {
    allocation->fd = gst_dmabuf_memory_get_fd(mem);
    allocation->flags = Allocation::Usage::DMABUF;
    allocation->size = gst_memory_get_sizes(mem, &offset, &maxsize);
  } else {
    allocation->fd = -1;
    allocation->flags = Allocation::Usage::MMAP;
    allocation->size = gst_buffer_get_size(gstbuf);
  }
  allocation->offset = static_cast<off_t>(offset);

  // Prefer the caps on the sample: for the decoder these track mid-stream resolution
  // changes, which is what makes source-change handling free on this backend.
  allocation->view = MemoryView::create(fallbackFormat, 0, 0);
  if (auto * caps = gst_sample_get_caps(sample)) {
    GstVideoInfo info;
    gst_video_info_init(&info);
    if (gst_video_info_from_caps(&info, caps)) {
      allocation->view = MemoryView::create(
          fallbackFormat, GST_VIDEO_INFO_WIDTH(&info), GST_VIDEO_INFO_HEIGHT(&info));
    } else if (auto * s = gst_caps_get_structure(caps, 0)) {
      gint width = 0;
      gint height = 0;
      if (gst_structure_get_int(s, "width", &width) &&
          gst_structure_get_int(s, "height", &height)) {
        allocation->view = MemoryView::create(fallbackFormat, width, height);
      }
    }
  }
  // Compressed output is a single opaque plane whose length is what the codec produced.
  if (allocation->view.num_planes <= 1) {
    allocation->view.num_planes = 1;
    allocation->view.planes[0].size = allocation->size;
  }

  std::shared_ptr<Memory> memory;
  if (isDmabuf) {
    memory = std::make_shared<GstreamerMemory>(allocation);
  } else {
    auto system = std::make_shared<GstreamerSystemMemory>(allocation, gstbuf);
    if (not system->isMapped()) {
      LOGE("%s: failed to map system-memory output", __func__);
      return nullptr;
    }
    memory = std::move(system);
  }

  auto * owned = gst_sample_ref(sample);
  auto * raw = new Buffer();
  raw->memory = std::move(memory);
  raw->type = allocation->view.type;
  raw->index = index;
  raw->sequence = index;
  raw->bytesused = gst_buffer_get_size(gstbuf);
  raw->offset = static_cast<off_t>(offset);
  if (GST_BUFFER_PTS_IS_VALID(gstbuf)) {
    const auto pts = GST_BUFFER_PTS(gstbuf);
    raw->timestamp.tv_sec = static_cast<time_t>(pts / GST_SECOND);
    raw->timestamp.tv_usec = static_cast<suseconds_t>((pts % GST_SECOND) / GST_USECOND);
  }
  if (GST_BUFFER_FLAG_IS_SET(gstbuf, GST_BUFFER_FLAG_MARKER)) {
    raw->setEOS();
  }

  // Releasing the Buffer releases the sample, which returns the underlying V4L2 buffer to
  // the codec's capture queue.
  return std::shared_ptr<Buffer>(raw, [owned](Buffer * b) {
    delete b;
    gst_sample_unref(owned);
  });
}

GstBuffer * wrapBuffer(GstAllocator * allocator,
    const std::shared_ptr<Buffer> & item,
    const MemoryView & view,
    bool attachVideoMeta)
{
  const int fd = item->fd();
  if (fd < 0) {
    LOGE("%s: input buffer has no dmabuf fd", __func__);
    return nullptr;
  }

  const size_t size = item->size() > 0 ? item->size() : item->length();
  const int ownedFd = fcntl(fd, F_DUPFD_CLOEXEC, 0);
  if (ownedFd < 0) {
    LOGE("%s: failed to dup fd %d", __func__, fd);
    return nullptr;
  }
  auto * mem = gst_dmabuf_allocator_alloc(allocator, ownedFd, size);
  if (mem == nullptr) {
    LOGE("%s: failed to wrap fd %d as dmabuf memory", __func__, fd);
    close(ownedFd);
    return nullptr;
  }

  auto * gstbuf = gst_buffer_new();
  gst_buffer_append_memory(gstbuf, mem);
  gst_buffer_resize(gstbuf, 0, static_cast<gssize>(item->length() > 0 ? item->length() : size));

  if (attachVideoMeta) {
    const GstVideoFormat fmt = toGstVideoFormat(view.pixelfmt);
    if (fmt != GST_VIDEO_FORMAT_UNKNOWN) {
      // One dmabuf, but several video planes within it. n_memory stays 1, which is what the
      // driver's num_planes (INPUT_PLANES) expects; the plane layout is described here.
      gsize offsets[GST_VIDEO_MAX_PLANES] = { 0 };
      gint strides[GST_VIDEO_MAX_PLANES] = { 0 };
      gsize running = 0;
      const guint planes = view.num_planes > 0 ? view.num_planes : 1;
      for (guint i = 0; i < planes && i < GST_VIDEO_MAX_PLANES; i++) {
        offsets[i] = running;
        strides[i] = static_cast<gint>(view.planes[i].stride);
        running += view.planes[i].size;
      }
      gst_buffer_add_video_meta_full(gstbuf, GST_VIDEO_FRAME_FLAG_NONE, fmt, view.width,
          view.height, planes, offsets, strides);
    }
  }

  const auto ts = item->getTimestamp();
  GST_BUFFER_PTS(gstbuf) = static_cast<GstClockTime>(ts.tv_sec) * GST_SECOND +
                           static_cast<GstClockTime>(ts.tv_usec) * GST_USECOND;
  GST_BUFFER_DTS(gstbuf) = GST_CLOCK_TIME_NONE;

  // Hold the producer's Buffer alive until the codec is done with this GstBuffer.
  auto * retained = new std::shared_ptr<Buffer>(item);
  gst_mini_object_set_qdata(GST_MINI_OBJECT(gstbuf),
      g_quark_from_static_string("qrb-video-source-buffer"), retained,
      [](gpointer data) { delete static_cast<std::shared_ptr<Buffer> *>(data); });

  return gstbuf;
}
}  // namespace qrb::video_v4l2
