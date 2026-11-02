/*
 **************************************************************************************************
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 **************************************************************************************************
 */

#ifndef QRB_VIDEO_V4L2__V4L2MEMORY_HPP_
#define QRB_VIDEO_V4L2__V4L2MEMORY_HPP_

#include <fcntl.h>
#include <linux/videodev2.h>
#include <unistd.h>

#include <string>
#include <utility>

// DmabufAllocator/DmaAllocation used to be declared here. They are dma_heap wrappers with
// nothing V4L2-specific about them, so they now live in the common headers; kept included
// here because this backend allocates both its ports through them.
#include "DmabufAllocator.hpp"
#include "Memory.hpp"

namespace qrb::video_v4l2
{
class V4l2Memory : public Memory
{
public:
  explicit V4l2Memory(const std::shared_ptr<Allocation> & allocation) : Memory(allocation) {}

  V4l2Memory() = delete;
  ~V4l2Memory() override = default;

private:
  Type type;
};
}  // namespace qrb::video_v4l2

#endif  // QRB_VIDEO_V4L2__V4L2MEMORY_HPP_
