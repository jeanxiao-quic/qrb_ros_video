/*
 **************************************************************************************************
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 **************************************************************************************************
 */

#ifndef QRB_VIDEO_V4L2__GSTREAMERMEMORY_HPP_
#define QRB_VIDEO_V4L2__GSTREAMERMEMORY_HPP_

#include <gst/gst.h>
#include <gst/video/video.h>

#include <memory>

#include "Buffer.hpp"
#include "Memory.hpp"

// Note on naming: these types are deliberately NOT called GstMemory/GstBuffer/GstCodec.
// They live in namespace qrb::video_v4l2, so such a name would shadow GStreamer's own
// global C types for every unqualified use inside the namespace.
namespace qrb::video_v4l2
{
// Memory backed by a dmabuf fd owned by a GstSample that we hold a ref to. The base
// Memory::map() already mmap()s allocation->fd, which is what the CompressedImage path in
// the ROS encoder node relies on, so nothing needs overriding there.
class GstreamerMemory : public Memory
{
public:
  explicit GstreamerMemory(const std::shared_ptr<Allocation> & allocation) : Memory(allocation) {}

  GstreamerMemory() = delete;
  ~GstreamerMemory() override = default;
};

// Wraps an output GstSample as a Buffer. The sample -- not merely its GstBuffer -- is kept
// alive for as long as the returned Buffer, because the caps that describe the frame hang
// off the sample and the decoder's resolution can change between samples.
//
// Returns nullptr if the sample's first memory is not a dmabuf, which is what happens when
// capture-io-mode was not honoured; callers should treat that as a hard error rather than
// falling back, so the condition stays visible.
std::shared_ptr<Buffer> wrapSample(GstSample * sample, Format fallbackFormat, size_t index);

// Wraps an input Buffer's dmabuf fd as a GstBuffer ready for gst_app_src_push_buffer().
//
// The fd is adopted with GST_FD_MEMORY_FLAG_DONT_CLOSE so ownership stays with the
// producer (matching the non-owning Allocation the ROS nodes build), and a ref to `item`
// is attached as qdata so the producer gets its buffer back only once the codec is done
// with it -- the same contract BufferPool's alias-deleter provides on the V4L2 path.
GstBuffer * wrapBuffer(GstAllocator * allocator,
    const std::shared_ptr<Buffer> & item,
    const MemoryView & view,
    bool attachVideoMeta);

// Maps our pixel formats onto GStreamer's. Returns GST_VIDEO_FORMAT_UNKNOWN for the
// compressed ones, which are described by their own caps instead.
GstVideoFormat toGstVideoFormat(Format format);

// Media type string for a compressed format, e.g. "video/x-h264". Null for raw formats.
const char * toGstMediaType(Format format);
}  // namespace qrb::video_v4l2

#endif  // QRB_VIDEO_V4L2__GSTREAMERMEMORY_HPP_
