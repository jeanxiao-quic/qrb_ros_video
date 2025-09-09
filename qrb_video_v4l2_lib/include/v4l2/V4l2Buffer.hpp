/*
 **************************************************************************************************
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 **************************************************************************************************
 */

#ifndef QRB_VIDEO_V4L2__V4L2BUFFER_HPP_
#define QRB_VIDEO_V4L2__V4L2BUFFER_HPP_
#include <linux/videodev2.h>

#include <linux/v4l2_vidc_extensions.hpp>
#include <memory>

#include "BufferChannel.hpp"
#include "V4l2Memory.hpp"

namespace qrb::video_v4l2
{
class V4l2Buffer : public Buffer
{
public:
  static std::shared_ptr<V4l2Buffer> create(const std::shared_ptr<Allocation> & allocation)
  {
    const auto buffer = std::shared_ptr<V4l2Buffer>();
    buffer->type = allocation->view.type;
    buffer->memory = std::make_shared<V4l2Memory>(allocation);
    return buffer;
  }

  explicit V4l2Buffer(const Buffer & buffer) : Buffer(buffer) { init(); }

  explicit operator v4l2_buffer() const;

private:
  v4l2_buffer * v4l2_buffer_;
  v4l2_plane * v4l2_mplane_;

  void init()
  {
    v4l2_mplane_ = static_cast<v4l2_plane *>(std::calloc(BUFFER_MAX_PLANES, sizeof(v4l2_plane)));
    v4l2_buffer_ = static_cast<v4l2_buffer *>(std::calloc(1, sizeof(v4l2_buffer)));
    if (v4l2_buffer_ == nullptr || v4l2_mplane_ == nullptr) {
      throw std::bad_alloc();
    }
    v4l2_buffer_->m.planes = v4l2_mplane_;
  }

  V4l2Buffer() { init(); }
};
}  // namespace qrb::video_v4l2

#endif  // QRB_VIDEO_V4L2__V4L2BUFFER_HPP_
