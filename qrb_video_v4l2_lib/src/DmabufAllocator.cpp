/*
**************************************************************************************************
* Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
**************************************************************************************************
*/

#include "DmabufAllocator.hpp"

#include <linux/dma-heap.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <stdexcept>

namespace qrb::video_v4l2
{
DmabufAllocator::~DmabufAllocator()
{
  if (fd != -1)
    ::close(fd);
}

std::shared_ptr<Allocation> DmabufAllocator::allocate()
{
  struct dma_heap_allocation_data data = {};
  data.len = view.planes[0].size;
  data.fd = 0;
  data.fd_flags = O_RDWR | O_CLOEXEC;
  data.heap_flags = 0;
  if (::ioctl(fd, DMA_HEAP_IOCTL_ALLOC, &data) < 0) {
    throw std::runtime_error("Failed to allocate buffer");
  }

  return from(data.fd, data.len);
}

void DmabufAllocator::set(const MemoryView & view)
{
  type = Memory::determineType(view.pixelfmt);
  this->view = view;
}

std::shared_ptr<DmaAllocation> DmabufAllocator::from(int memfd, size_t size) const
{
  auto allocation = std::make_shared<DmaAllocation>();
  allocation->fd = memfd;
  allocation->size = size;
  allocation->offset = 0;
  allocation->view = view;
  allocation->flags = Allocation::Usage::DMABUF;
  return allocation;
}
};  // namespace qrb::video_v4l2
