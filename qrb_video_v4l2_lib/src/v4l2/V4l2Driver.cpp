/*
**************************************************************************************************
* Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
**************************************************************************************************
*/

#include "V4l2Driver.hpp"

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cstdint>
#include <filesystem>
#include <functional>
#include <linux/v4l2_vidc_extensions.hpp>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>

#include "Log.hpp"

namespace qrb::video_v4l2
{
static std::map<uint32_t, std::string> ctrl_mapping = {
  { V4L2_CID_MIN_BUFFERS_FOR_CAPTURE, "min output" },
  { V4L2_CID_MIN_BUFFERS_FOR_OUTPUT, "min input" },
  { V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE, "Frame RC Enable" },
  { V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP, "I Frame QP" },
  { V4L2_CID_MPEG_VIDEO_HEVC_I_FRAME_QP, "I Frame QP" },
  { V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP, "P Frame QP" },
  { V4L2_CID_MPEG_VIDEO_HEVC_P_FRAME_QP, "P Frame QP" },
  { V4L2_CID_MPEG_VIDEO_H264_B_FRAME_QP, "B Frame QP" },
  { V4L2_CID_MPEG_VIDEO_HEVC_B_FRAME_QP, "B Frame QP" },
  { V4L2_CID_MPEG_VIDEO_H264_I_FRAME_MAX_QP, "Max I Frame QP" },
  { V4L2_CID_MPEG_VIDEO_HEVC_I_FRAME_MAX_QP, "Max I Frame QP" },
  { V4L2_CID_MPEG_VIDEO_H264_P_FRAME_MAX_QP, "Max P Frame QP" },
  { V4L2_CID_MPEG_VIDEO_HEVC_P_FRAME_MAX_QP, "Max P Frame QP" },
  { V4L2_CID_MPEG_VIDEO_H264_B_FRAME_MAX_QP, "Max B Frame QP" },
  { V4L2_CID_MPEG_VIDEO_HEVC_B_FRAME_MAX_QP, "Max B Frame QP" },
  { V4L2_CID_MPEG_VIDEO_H264_I_FRAME_MIN_QP, "Min I Frame QP" },
  { V4L2_CID_MPEG_VIDEO_HEVC_I_FRAME_MIN_QP, "Min I Frame QP" },
  { V4L2_CID_MPEG_VIDEO_H264_P_FRAME_MIN_QP, "Min P Frame QP" },
  { V4L2_CID_MPEG_VIDEO_HEVC_P_FRAME_MIN_QP, "Min P Frame QP" },
  { V4L2_CID_MPEG_VIDEO_H264_B_FRAME_MIN_QP, "Min B Frame QP" },
  { V4L2_CID_MPEG_VIDEO_HEVC_B_FRAME_MIN_QP, "Min B Frame QP" },
  { V4L2_CID_MPEG_VIDEO_H264_8X8_TRANSFORM, "H.264 8x8 Transform" },
  { V4L2_CID_MPEG_VIDEO_PREPEND_SPSPPS_TO_IDR, "Prepend SPSPPS to IDR" },
  { V4L2_CID_MPEG_VIDEO_HEVC_PROFILE, "HEVC Profile" },
  { V4L2_CID_MPEG_VIDEO_HEVC_LEVEL, "HEVC Level" },
  { V4L2_CID_MPEG_VIDEO_HEVC_TIER, "HEVC Tier" },
  { V4L2_CID_MPEG_VIDEO_H264_PROFILE, "H.264 Profile" },
  { V4L2_CID_MPEG_VIDEO_H264_LEVEL, "H.264 Level" },
  { V4L2_CID_MPEG_VIDEO_H264_ENTROPY_MODE, "H.264 EntropyCoding" },
  { V4L2_CID_MPEG_VIDEO_HEADER_MODE, "Header Mode" },
  { V4L2_CID_MPEG_VIDEO_BITRATE, "Bitrate" },
  { V4L2_CID_MPEG_VIDEO_BITRATE_MODE, "Bitrate Mode" },
  { V4L2_CID_MPEG_VIDEO_H264_HIERARCHICAL_CODING_TYPE, "Hierarchical Coding Type" },
  { V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_TYPE, "Hierarchical Coding Type" },
  { V4L2_CID_MPEG_VIDEO_H264_HIERARCHICAL_CODING_LAYER, "Hierarchical Coding Layer" },
  { V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_LAYER, "Hierarchical Coding Layer" },
  { V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L0_BR, "Hierarchical Coding Bitrate" },
  { V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L1_BR, "Hierarchical Coding Bitrate" },
  { V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L2_BR, "Hierarchical Coding Bitrate" },
  { V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L3_BR, "Hierarchical Coding Bitrate" },
  { V4L2_CID_ROTATE, "Rotate" },
  { V4L2_CID_HFLIP, "HFlip" },
  { V4L2_CID_VFLIP, "VFlip" },
  { V4L2_CID_MPEG_VIDEO_GOP_SIZE, "GOP Size" },
  { V4L2_CID_MPEG_VIDEO_B_FRAMES, "B Frames" },
  { V4L2_CID_MPEG_VIDEO_VBV_DELAY, "VBV Delay" },
  { V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MODE, "Multi-Slice Mode" },
  { V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_MB, "Multi-Slice Max MB" },
  { V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_BYTES, "Multi-Slice Max Bytes" },
  { V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_MODE, "Loop Filter Mode" },
  { V4L2_CID_MPEG_VIDEO_HEVC_LOOP_FILTER_MODE, "Loop Filter Mode" },
  { V4L2_CID_MPEG_VIDEO_LTR_COUNT, "LTR Count" },
  { V4L2_CID_MPEG_VIDEO_FRAME_LTR_INDEX, "Frame LTR Index" },
  { V4L2_CID_MPEG_VIDEO_USE_LTR_FRAMES, "Use LTR Frame" },
  { V4L2_CID_MPEG_VIDEO_INTRA_REFRESH_PERIOD, "Intra Refresh Period" },
  { V4L2_CID_MPEG_VIDEO_INTRA_REFRESH_PERIOD_TYPE, "Intra Refresh Type" },
  { V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME, "Force Key Frame" },
};

const char * ctrl_name(const int id)
{
  try {
    const char * name = ctrl_mapping.at(id).c_str();
    return name;
  } catch (std::out_of_range & e) {
    return "unknown";
  }
}

V4l2Driver::V4l2Driver(std::string & name)
  : devName(std::move(name)), pollThreadRunning(false), pollThreadPaused(false)
{
  auto devPath = std::filesystem::path(devName);
  if (not devPath.is_absolute()) {
    devPath = std::filesystem::path("/dev") / devPath;
  }
  devfd = open(devPath.c_str(), O_RDWR);
  if (devfd < 0) {
    throw std::runtime_error("Failed to open device");
  }
}

void V4l2Driver::start()
{
  if (not pollThread.joinable()) {
    pollThreadRunning = true;
    pollThread = std::thread(std::mem_fn(&V4l2Driver::threadLoop), this);
  }
  LOGI("poll thread started");
}

void V4l2Driver::stop()
{
  if (pollThreadRunning) {
    pollThreadRunning = false;
    pollThread.join();
  }
  LOGI("poll thread stopped");
}

void V4l2Driver::pause()
{
  if (pollThreadRunning && not pollThreadPaused)
    pollThreadPaused = true;
  LOGI("poll thread paused");
}

void V4l2Driver::resume()
{
  if (pollThreadPaused) {
    pollThreadPaused = false;
    pauseNotifier.notify_one();
  }
  LOGI("poll thread resumed");
}

void V4l2Driver::threadLoop()
{
  v4l2_buffer buffer{};
  v4l2_plane plane[INPUT_PLANES];
  v4l2_event event{};
  pollfd pollFds[2];

  pollFds[0].events = POLLIN | POLLRDNORM | POLLOUT | POLLWRNORM | POLLRDBAND | POLLPRI | POLLERR;
  pollFds[0].fd = devfd;

  while (pollThreadRunning) {
    if (pollThreadPaused) {
      std::unique_lock<std::mutex> lock(pauseMutex);
      pauseNotifier.wait(lock);
    }
    int ret = poll(pollFds, 1, 1000);
    if (ret == -ETIMEDOUT) {
      LOGW("V4l2Driver: poll timedout\n");
      continue;
    } else if (ret < 0 && errno != EINTR && errno != EAGAIN) {
      LOGE("V4l2Driver: poll error %d\n", ret);
      callback->onV4l2Error(EAGAIN);
      break;
    }
    if (pollFds[0].revents & POLLERR) {
      LOGI("V4l2Driver: poll error received\n");
      mError = true;
      callback->onV4l2Error(POLLERR);
      break;
    }
    if (pollFds[0].revents & POLLPRI) {
      LOGI("V4l2Driver: PRI received.\n");
      memset(&event, 0, sizeof(event));
      if (!ioctl(devfd, VIDIOC_DQEVENT, &event)) {
        LOGI("V4l2Driver: Received v4l2 event, type %#x\n", event.type);
        callback->onV4l2EventDone(&event);
      }
    }
    if ((pollFds[0].revents & POLLIN) || (pollFds[0].revents & POLLRDNORM)) {
      LOGI("V4l2Driver: IN/RDNORM received.\n");
      do {
        memset(&buffer, 0, sizeof(buffer));
        memset(plane, 0, sizeof(plane));
        buffer.type = OUTPUT_MPLANE;
        buffer.m.planes = plane;
        buffer.length = 1;
        buffer.memory = V4L2_MEMORY_DMABUF;
        if (ioctl(devfd, VIDIOC_DQBUF, &buffer)) {
          break;
        }

        if (!callback->onV4l2BufferDone(&buffer)) {
          mError = true;
        }
      } while (true);
    }
    if ((pollFds[0].revents & POLLOUT) || (pollFds[0].revents & POLLWRNORM)) {
      LOGI("V4l2Driver: OUT/WRNORM received.\n");
      do {
        memset(&buffer, 0, sizeof(buffer));
        memset(plane, 0, sizeof(plane));
        buffer.type = INPUT_MPLANE;
        buffer.m.planes = plane;
        buffer.length = 1;
        buffer.memory = V4L2_MEMORY_DMABUF;
        if (ioctl(devfd, VIDIOC_DQBUF, &buffer)) {
          break;
        }
        if (!callback->onV4l2BufferDone(&buffer)) {
          mError = true;
        }
      } while (true);
    }
  }
  LOGI("V4l2Driver::threadLoop() ends.\n");
}

int V4l2Driver::streamOn(int port) const
{
  LOGI("streamon: port %d\n", port);
  int ret = ioctl(devfd, VIDIOC_STREAMON, &port);
  if (ret) {
    LOGE("streamon failed for port %d\n", port);
    return -EINVAL;
  }
  return 0;
}

int V4l2Driver::streamOff(int port) const
{
  LOGI("streamoff: port %d\n", port);
  int ret = ioctl(devfd, VIDIOC_STREAMOFF, &port);
  if (ret) {
    LOGE("streamoff failed for port %d\n", port);
    return -EINVAL;
  }
  return 0;
}

int V4l2Driver::getFormat(v4l2_format * fmt) const
{
  int ret = ioctl(devfd, VIDIOC_G_FMT, fmt);
  if (ret) {
    LOGE("getFormat failed for type %d\n", fmt->type);
    return -EINVAL;
  }
  LOGI("getFormat: type %d, [wxh] %dx%d, fmt %#x, size %d\n", fmt->type, fmt->fmt.pix_mp.width,
      fmt->fmt.pix_mp.height, fmt->fmt.pix_mp.pixelformat, fmt->fmt.pix_mp.plane_fmt[0].sizeimage);
  return 0;
}

int V4l2Driver::setCodecPixelFmt(uint32_t planeType, uint32_t codecPixFmt) const
{
  struct v4l2_fmtdesc fmtdesc;
  struct v4l2_format fmt;
  bool found = false;
  int ret = 0;

  /* check if driver supports client requested fomat */
  memset(&fmtdesc, 0, sizeof(fmtdesc));
  fmtdesc.index = 0;
  fmtdesc.type = planeType;
  while (!ret) {
    ret = ioctl(devfd, VIDIOC_ENUM_FMT, &fmtdesc);
    if (ret) {
      break;
    }
    if (fmtdesc.pixelformat == codecPixFmt) {
      found = true;
      break;
    }
    fmtdesc.index++;
  }
  if (!found) {
    LOGE("client format %#x not supported\n", codecPixFmt);
    return -EINVAL;
  }

  memset(&fmt, 0, sizeof(fmt));
  fmt.type = planeType;
  ret = ioctl(devfd, VIDIOC_G_FMT, &fmt);
  if (ret) {
    LOGE("getFormat failed for type %d\n", fmt.type);
    return -EINVAL;
  }
  fmt.fmt.pix_mp.pixelformat = codecPixFmt;

  ret = ioctl(devfd, VIDIOC_S_FMT, &fmt);
  if (ret) {
    LOGE("setFormat failed for type %d\n", fmt.type);
    return -EINVAL;
  }
  LOGI("pixelCodecFmt: type %d, [wxh] %dx%d, fmt %#x, size %d\n", fmt.type, fmt.fmt.pix_mp.width,
      fmt.fmt.pix_mp.height, fmt.fmt.pix_mp.pixelformat, fmt.fmt.pix_mp.plane_fmt[0].sizeimage);

  return ret;
}

int V4l2Driver::getSelection(v4l2_selection * sel) const
{
  int ret = ioctl(devfd, VIDIOC_G_SELECTION, sel);
  if (ret) {
    LOGE("getSelection failed for type %d, target %d\n", sel->type, sel->target);
    return -EINVAL;
  }
  LOGI("getSelection: type %d, target %d, left %d top %d width %d height %d\n", sel->type,
      sel->target, sel->r.left, sel->r.top, sel->r.width, sel->r.height);
  return 0;
}

int V4l2Driver::getControl(v4l2_control * ctrl) const
{
  int ret = ioctl(devfd, VIDIOC_G_CTRL, ctrl);
  if (ret) {
    LOGE("getCotrol failed for \"%s\"\n", ctrl_name(ctrl->id));
    return -EINVAL;
  }
  LOGI("getControl: \"%s\", value %d\n", ctrl_name(ctrl->id), ctrl->value);
  return 0;
}

int V4l2Driver::setControl(v4l2_control * ctrl) const
{
  LOGI("setControl: \"%s\", value %d\n", ctrl_name(ctrl->id), ctrl->value);
  int ret = ioctl(devfd, VIDIOC_S_CTRL, ctrl);
  if (ret) {
    LOGE("setCotrol failed for \"%s\"\n", ctrl_name(ctrl->id));
    return -EINVAL;
  }

  return 0;
}

int V4l2Driver::reqBufs(struct v4l2_requestbuffers * reqbufs) const
{
  int ret = 0;

  LOGI("reqBufs: type %d, count %d memory %d\n", reqbufs->type, reqbufs->count, reqbufs->memory);
  ret = ioctl(devfd, VIDIOC_REQBUFS, reqbufs);
  if (ret) {
    LOGE("reqBufs failed for type %d, count %d memory %d\n", reqbufs->type, reqbufs->count,
        reqbufs->memory);
    return -EINVAL;
  }
  return 0;
}

int V4l2Driver::queueBuf(v4l2_buffer * buf) const
{
  int ret = ioctl(devfd, VIDIOC_QBUF, buf);
  if (ret) {
    LOGE("failed to QBUF: %s\n", strerror(ret));
    return -EINVAL;
  }

  return 0;
}

int V4l2Driver::decCommand(v4l2_decoder_cmd * cmd) const
{
  int ret = ioctl(devfd, VIDIOC_DECODER_CMD, cmd);
  if (ret) {
    LOGE("decCommand: error %d\n", ret);
    return -EINVAL;
  }
  return 0;
}

int V4l2Driver::encCommand(v4l2_encoder_cmd * cmd) const
{
  int ret = ioctl(devfd, VIDIOC_ENCODER_CMD, cmd);
  if (ret) {
    LOGE("encCommand: error %d\n", ret);
    return -EINVAL;
  }
  return 0;
}

int V4l2Driver::setFormat(struct v4l2_format * fmt) const
{
  int ret = ioctl(devfd, VIDIOC_S_FMT, fmt);
  if (ret) {
    LOGE("setFormat failed for type %d\n", fmt->type);
    return -EINVAL;
  }
  LOGI("setFormat: type %d, [wxh] %dx%d, fmt %#x, size %d\n", fmt->type, fmt->fmt.pix_mp.width,
      fmt->fmt.pix_mp.height, fmt->fmt.pix_mp.pixelformat, fmt->fmt.pix_mp.plane_fmt[0].sizeimage);
  return 0;
}

int V4l2Driver::setParam(v4l2_streamparm * sparm) const
{
  int ret = ioctl(devfd, VIDIOC_S_PARM, sparm);
  if (ret) {
    LOGE("setParm failed for type %u\n", sparm->type);
    return -EINVAL;
  }
  return 0;
}

int V4l2Driver::setSelection(v4l2_selection * sel) const
{
  int ret = ioctl(devfd, VIDIOC_S_SELECTION, sel);
  if (ret) {
    LOGE("setSelection failed for type %d, target %d\n", sel->type, sel->target);
    return -EINVAL;
  }
  LOGI("setSelection: type %d, target %d, left %d top %d width %d height %d\n", sel->type,
      sel->target, sel->r.left, sel->r.top, sel->r.width, sel->r.height);
  return 0;
}

int V4l2Driver::queryCapabilities(v4l2_capability * caps) const
{
  int ret = ioctl(devfd, VIDIOC_QUERYCAP, caps);
  if (ret) {
    LOGE("Failed to query capabilities\n");
    return -EINVAL;
  }
  LOGI(
      "queryCapabilities: driver name: %s, card: %s, bus_info: %s, "
      "version: %#x, capabilities: %#x, device_caps: %#x \n",
      caps->driver, caps->card, caps->bus_info, caps->version, caps->capabilities,
      caps->device_caps);
  return 0;
}

int V4l2Driver::queryMenu(v4l2_querymenu * querymenu) const
{
  int ret = ioctl(devfd, VIDIOC_QUERYMENU, querymenu);
  if (ret) {
    LOGE("Failed to query menu: %s\n", ctrl_name(querymenu->id));
    return -EINVAL;
  }
  LOGI("queryMenu: name: \"%s\", id: %#x, index: %d\n", querymenu->name, querymenu->id,
      querymenu->index);
  return 0;
}

int V4l2Driver::queryControl(v4l2_queryctrl * ctrl) const
{
  int ret = ioctl(devfd, VIDIOC_QUERYCTRL, ctrl);
  if (ret) {
    LOGE("Failed to query ctrl: %s\n", ctrl_name(ctrl->id));
    return -EINVAL;
  }
  LOGI("queryCotrol: name: \"%s\", min: %d, max: %d, step: %d, default: %d\n", ctrl->name,
      ctrl->minimum, ctrl->maximum, ctrl->step, ctrl->default_value);
  return 0;
}

int V4l2Driver::enumFormat(v4l2_fmtdesc * fmtdesc) const
{
  int ret = ioctl(devfd, VIDIOC_ENUM_FMT, fmtdesc);
  if (ret) {
    LOGE("enumFormat ended for index %d\n", fmtdesc->index);
    return -ENOTSUP;
  }
  LOGI(
      "enumFormat: index %d, description: \"%s\", pixelFmt: %#x, flags: "
      "%#x\n",
      fmtdesc->index, fmtdesc->description, fmtdesc->pixelformat, fmtdesc->flags);
  return 0;
}

int V4l2Driver::enumFramesize(v4l2_frmsizeenum * frmsize) const
{
  int ret = ioctl(devfd, VIDIOC_ENUM_FRAMESIZES, frmsize);
  if (ret) {
    LOGE("enumFramesize failed for pixel_format %#x\n", frmsize->pixel_format);
    return -EINVAL;
  }
  if (frmsize->type != V4L2_FRMSIZE_TYPE_STEPWISE) {
    LOGE("enumFramesize: type (%d) returned in not stepwise\n", frmsize->type);
    return -EINVAL;
  }
  LOGI("enumFramesize: [%u x %u] to [%u x %u]\n", frmsize->stepwise.min_width,
      frmsize->stepwise.min_height, frmsize->stepwise.max_width, frmsize->stepwise.max_height);
  return 0;
}

int V4l2Driver::enumFrameInterval(v4l2_frmivalenum * fival) const
{
  int ret = ioctl(devfd, VIDIOC_ENUM_FRAMEINTERVALS, fival);
  if (ret) {
    LOGE("enumFrameInterval failed for pixel_format %#x\n", fival->pixel_format);
    return -EINVAL;
  }
  if (fival->type != V4L2_FRMIVAL_TYPE_STEPWISE) {
    LOGE("enumFramesize: type (%d) returned in not stepwise\n", fival->type);
    return -EINVAL;
  }
  LOGI(
      "enumFrameInterval: resoltion [%u x %u], interval [%u / %u] to [%u / "
      "%u]\n",
      fival->width, fival->height, fival->stepwise.min.numerator, fival->stepwise.min.denominator,
      fival->stepwise.max.numerator, fival->stepwise.max.denominator);

  return ret;
}

int V4l2Driver::subscribeEvent(unsigned int event_type) const
{
  int ret = 0;
  struct v4l2_event_subscription event;
  memset(&event, 0, sizeof(event));
  event.type = event_type;
  LOGI("subscribeEvent: type %d\n", event_type);
  ret = ioctl(devfd, VIDIOC_SUBSCRIBE_EVENT, &event);
  if (ret) {
    LOGE("subscribeEvent: error %d\n", ret);
    return ret;
  }
  return 0;
}

int V4l2Driver::unsubscribeEvent(unsigned int event_type) const
{
  int ret = 0;
  struct v4l2_event_subscription event;
  memset(&event, 0, sizeof(event));
  event.type = event_type;
  LOGI("unsubscribeEvent: type %d\n", event_type);
  ret = ioctl(devfd, VIDIOC_UNSUBSCRIBE_EVENT, &event);
  if (ret) {
    LOGE("unsubscribeEvent: error %d\n", ret);
    return ret;
  }
  return 0;
}

int V4l2Driver::registerCallbacks(Callback * cb)
{
  if (not callback)
    callback = cb;
  return 0;
}
}  // namespace qrb::video_v4l2
