/*
**************************************************************************************************
* Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
**************************************************************************************************
*/

#include <Decoder.hpp>
#include <qrb_ros_transport_image_type/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "video_node.hpp"

namespace qrb_ros
{
namespace video
{
using namespace qrb::video_v4l2;

template <typename InputMessageT, typename OutputMessageT>
class VideoDecoder : public VideoNode<InputMessageT, OutputMessageT>, public VideoCodec::Notifier
{
public:
  explicit VideoDecoder(const rclcpp::NodeOptions & options)
    : VideoNode<InputMessageT, OutputMessageT>("ros2video_decoder", options)
  {
    if (this->format == "h265")
      decoder_ = Decoder::create(MIME_H265);
    else
      decoder_ = Decoder::create(MIME_H264);
    cb_.reset(this, [](auto * p) {});
    Format f = this->pixel_format == "p010" ? Format::P010 : Format::NV12;
    decoder_->configure(Setting::create(f));
    decoder_->setNotifier(cb_);
    decoder_->start();
    RCLCPP_INFO(this->get_logger(), "VideoDecoder has been started.");
  }

  using Type = std::conditional_t<std::is_same_v<OutputMessageT, transport::type::Image>,
      transport::type::Image,
      sensor_msgs::msg::Image>;

  bool onBufferAvailable(const std::shared_ptr<Buffer> & item) override
  {
    RCLCPP_DEBUG(this->get_logger(), "received output buffer: [%ld]", item->index);
    this->seqno++;
    publish_message_<Type>(item);
    RCLCPP_DEBUG(this->get_logger(), "output buffer: [%ld] publish completed", item->index);
    return true;
  }

protected:
  bool handle_message(const InputMessageT & msg) override
  {
    bool ret = this->handle_message_(msg);
    return ret;
  }

  virtual bool handle_message_(const transport::type::Image & msg)
  {
    auto allocation = std::make_shared<Allocation>();
    allocation->fd = dup(msg.dmabuf->fd());
    allocation->size = msg.dmabuf->size();
    allocation->flags = Allocation::Usage::DMABUF;
    allocation->offset = 0;
    allocation->view.pixelfmt = msg.encoding == "h265" ? Format::HEVC : Format::H264;
    allocation->view.width = msg.width;
    allocation->view.height = msg.height;
    allocation->view.num_planes = 1;
    auto buffer = Buffer::create(allocation);
    timeval timestamp = {};
    timestamp.tv_sec = msg.header.stamp.sec;
    timestamp.tv_usec = msg.header.stamp.nanosec / 1000L;
    buffer->setTimestamp(timestamp);
    buffer->bytesused = msg.dmabuf->size();
    buffer->offset = 0;
    decoder_->queueBuffer(buffer);
    RCLCPP_DEBUG(this->get_logger(),
        "[%s]: transport::type::Image, ts [%ld.%ld], fd %d, size %zd, flags %x, format %x, width "
        "%d, height %d",
        __func__, timestamp.tv_sec, timestamp.tv_usec, allocation->fd, allocation->size,
        allocation->flags, allocation->view.pixelfmt, allocation->view.width,
        allocation->view.height);
    return true;
  }

  virtual bool handle_message_(const sensor_msgs::msg::CompressedImage & msg)
  {
    auto buffer = decoder_->acquireBuffer();
    timeval timestamp = {};
    if (!buffer) {
      RCLCPP_ERROR(this->get_logger(), "Failed to acquire buffer");
      return false;
    }
    if (buffer->size() < msg.data.size()) {
      RCLCPP_ERROR(this->get_logger(), "Buffer size is too small");
      return false;
    }

    if (msg.data.size() == 0) {
      buffer->setEOS();
    } else {
      timestamp.tv_sec = msg.header.stamp.sec;
      timestamp.tv_usec = msg.header.stamp.nanosec / 1000L;
      buffer->setTimestamp(timestamp);

      buffer->memory->map();
      std::memcpy(buffer->data(), msg.data.data(),
          msg.data.size() * sizeof(sensor_msgs::msg::CompressedImage::_data_type::value_type));
      buffer->memory->unmap();
      buffer->bytesused = msg.data.size();
      buffer->offset = 0;
    }

    decoder_->queueBuffer(buffer);
    RCLCPP_DEBUG(this->get_logger(),
        "[%s]: transport::type::CompressedImage, ts [%ld.%ld], format %s size %zd", __func__,
        timestamp.tv_sec, timestamp.tv_usec, msg.format.c_str(), buffer->bytesused);
    return false;
  }

  template <typename T, std::enable_if_t<std::is_same_v<transport::type::Image, T>, bool> = true>
  bool publish_message_(std::shared_ptr<Buffer> item)
  {
    auto destroy_callback = [buf = item](auto * p) {
      // in case of destroying DmaBuffer, VideoCodec buffer is returned.
      delete p;
    };
    auto msg = std::make_unique<Type>();
    msg->header.stamp.sec = item->timestamp.tv_sec;
    msg->header.stamp.nanosec = item->timestamp.tv_usec * 1000L;
    msg->header.frame_id = std::to_string(this->seqno);
    auto view = item->view();
    msg->width = view.width;
    msg->height = view.height;
    auto fd = item->fd();
    size_t image_size = item->size();
    msg->dmabuf = std::shared_ptr<lib_mem_dmabuf::DmaBuffer>(
        new lib_mem_dmabuf::DmaBuffer(dup(fd), image_size), destroy_callback);
    msg->encoding = this->pixel_format;
    this->publisher->publish(std::move(msg));
    RCLCPP_DEBUG(this->get_logger(),
        "[%s]: transport::type::Image, ts [%ld.%ld], fd %d, format %s size %zd", __func__,
        item->timestamp.tv_sec, item->timestamp.tv_usec, fd, this->pixel_format.c_str(),
        image_size);
    return true;
  }

private:
  std::shared_ptr<Client> decoder_;
  std::shared_ptr<Notifier> cb_;
};

using Decoder = VideoDecoder<sensor_msgs::msg::CompressedImage, qrb_ros::transport::type::Image>;

template <typename InputMessageT, typename OutputMessageT = InputMessageT>
class PerfVideoDecoder : public VideoDecoder<InputMessageT, OutputMessageT>, public Stats
{
  using Super = VideoDecoder<InputMessageT, OutputMessageT>;

public:
  explicit PerfVideoDecoder(const rclcpp::NodeOptions & options)
    : Super(options), Stats(this->shared_from_this())
  {
    interval_ = this->declare_parameter("interval", 2000);
  }

  ~PerfVideoDecoder() override = default;

  bool onBufferAvailable(const std::shared_ptr<Buffer> & item) override
  {
    timeval t = item->timestamp;
    auto size = item->bytesused;
    Super::onBufferAvailable(item);
    queue_fbd(t, size);
    return true;
  }

protected:
  bool handle_message_(const sensor_msgs::msg::CompressedImage & msg) override
  {
    queue_etb({ msg.header.stamp.sec, msg.header.stamp.nanosec / 1000 }, msg.data.size());
    return Super::handle_message_(msg);
  }
};

using PerfDecoder =
    PerfVideoDecoder<sensor_msgs::msg::CompressedImage, qrb_ros::transport::type::Image>;
}  // namespace video
}  // namespace qrb_ros

RCLCPP_COMPONENTS_REGISTER_NODE(qrb_ros::video::Decoder);
RCLCPP_COMPONENTS_REGISTER_NODE(qrb_ros::video::PerfDecoder)
