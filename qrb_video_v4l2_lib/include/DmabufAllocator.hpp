/*
 **************************************************************************************************
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 **************************************************************************************************
 */

#ifndef QRB_VIDEO_V4L2__DMABUFALLOCATOR_HPP_
#define QRB_VIDEO_V4L2__DMABUFALLOCATOR_HPP_

#include <fcntl.h>
#include <unistd.h>

#include <string>

#include "Memory.hpp"

namespace qrb::video_v4l2
{
// Allocates from a kernel dma_heap. Nothing here is V4L2-specific -- it lived under v4l2/
// until the GStreamer backend needed it too, which would have made it unavailable whenever
// the V4L2 backend was compiled out.
struct DmaAllocation : Allocation
{
  ~DmaAllocation() override
  {
    if (fd != -1) {
      ::close(fd);
    }
  }

  DmaAllocation() = default;
};

class DmabufAllocator : public Allocator
{
public:
  DmabufAllocator(std::string name = "system") : Allocator(), name(name)
  {
    const auto dma_path = dma_dir + name;
    fd = ::open(dma_path.c_str(), O_RDWR);
  }

  ~DmabufAllocator() override;

  std::shared_ptr<Allocation> allocate() override;

  void set(const MemoryView & view) override;

private:
  [[nodiscard]] std::shared_ptr<DmaAllocation> from(int memfd, size_t size) const;

  std::string name;
  std::string dma_dir = "/dev/dma_heap/";
  Memory::Type type;
  int fd = -1;
};
}  // namespace qrb::video_v4l2

#endif  // QRB_VIDEO_V4L2__DMABUFALLOCATOR_HPP_
