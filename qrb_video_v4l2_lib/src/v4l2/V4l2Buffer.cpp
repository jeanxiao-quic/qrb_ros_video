/*
**************************************************************************************************
* Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
**************************************************************************************************
*/

#include "V4l2Buffer.hpp"

#include "V4l2Driver.hpp"

namespace qrb::video_v4l2
{
V4l2Buffer::operator v4l2_buffer() const
{
  v4l2_buffer & buf = *v4l2_buffer_;
  auto allocation = memory->getAllocation();
  auto & usage = allocation->flags;

  buf.index = index;
  buf.sequence = sequence;
  buf.timestamp = timestamp;
  auto view = memory->view();

  buf.length = view.num_planes;
  buf.memory = usage == Allocation::Usage::DMABUF ? V4L2_MEMORY_DMABUF : V4L2_MEMORY_MMAP;

  for (size_t i = 0; i < view.num_planes; i++) {
    usage == Allocation::Usage::DMABUF ? buf.m.planes[i].m.fd = allocation->fd :
                                         buf.m.planes[i].m.mem_offset = allocation->offset;
    buf.m.planes[i].bytesused = bytesused;
    buf.m.planes[i].length = view.planes[i].size;
    buf.m.planes[i].data_offset = offset;
  }
  if (flags == Flags::EOS) {
    buf.flags |= V4L2_BUF_FLAG_LAST;
  }
  return buf;
}
}  // namespace qrb::video_v4l2
