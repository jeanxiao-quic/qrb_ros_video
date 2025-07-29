/*
**************************************************************************************************
* Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
**************************************************************************************************
*/

#include <Encoder.hpp>
#include <array>
#include <qrb_ros_transport_image_type/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "video_node.hpp"

namespace qrb_ros
{
namespace video
{
using namespace qrb::video_v4l2;

template <typename InputMessageT, typename OutputMessageT = InputMessageT>
class VideoEncoder : public VideoNode<InputMessageT, OutputMessageT>, public VideoCodec::Notifier
{
public:
  explicit VideoEncoder(const rclcpp::NodeOptions & options)
    : VideoNode<InputMessageT, OutputMessageT>("ros2video_encoder", options)
    , profile(this->template declare_parameter<std::string>("profile", "high"))
    , level(this->template declare_parameter<std::string>("level", "5.0"))
    , bitrate(this->template declare_parameter<std::string>("bitrate", "10000000"))
    , rate_control(this->template declare_parameter<std::string>("rate-control", "variable"))
    , width(this->template declare_parameter<std::string>("width", "1920"))
    , height(this->template declare_parameter<std::string>("height", "1080"))
    , framerate(this->template declare_parameter<std::string>("framerate", "30"))
  {
    if (this->format == "h265")
      encoder_ = Encoder::create(MIME_H265);
    else
      encoder_ = Encoder::create(MIME_H264);

    cb_.reset(this, [](auto * p) {});
    encoder_->setNotifier(cb_);
    configure_encoder();
    encoder_->start();
    RCLCPP_INFO(this->get_logger(), "VideoEncoder has been started.");
  }

  ~VideoEncoder() override { encoder_->stop(); }

  using IType = std::conditional_t<std::is_same_v<InputMessageT, transport::type::Image>,
      transport::type::Image,
      sensor_msgs::msg::Image>;

  using OType = std::conditional_t<std::is_same_v<OutputMessageT, transport::type::Image>,
      transport::type::Image,
      sensor_msgs::msg::CompressedImage>;

  bool onBufferAvailable(const std::shared_ptr<Buffer> & item) override
  {
    publish_message_<OutputMessageT>(item);
    return true;
  }

protected:
  bool handle_message(const InputMessageT & msg) override
  {
    this->seqno++;
    bool ret = this->handle_message_(msg);
    return ret;
  }

  virtual bool check_framerate(const transport::type::Image & msg)
  {
    if (msg.header.frame_id.empty())
      return false;

    auto frame_id = msg.header.frame_id;
    
    // search @num/denom in frame_id
    // e.g. "123@30" or "123/4@30"
    std::regex re("@(\\d+)(?:/(\\d+))?");
    std::smatch match;
    if (std::regex_search(frame_id, match, re)) {
      int numerator = std::stoi(match[1]);
      int denominator = match[2].length() ? std::stoi(match[2]) : 1;
      int fps = static_cast<double>(numerator) / denominator;

      // Check if user set this parameter
      auto overrides = this->get_node_parameters_interface()->get_parameter_overrides();
      bool user_set = overrides.count("framerate") > 0;

      if (!user_set) {
        RCLCPP_INFO(this->get_logger(), "update framerate to %d", fps);
        this->framerate = std::to_string(fps);
      }
      RCLCPP_INFO(this->get_logger(), "check framerate to %d", fps);

      return this->framerate == std::to_string(fps);
    }
    RCLCPP_ERROR(this->get_logger(), "Invalid framerate format: %s", frame_id.c_str());
    return false;
  }

  virtual bool handle_message_(const transport::type::Image & msg)
  {
    std::call_once(check_once, [&]() {
        check_framerate(msg);
    });
    auto allocation = std::make_shared<Allocation>();
    allocation->fd = msg.dmabuf->fd();
    allocation->size = msg.dmabuf->size();
    allocation->flags = Allocation::Usage::DMABUF;
    allocation->offset = 0;
    allocation->view.pixelfmt = msg.encoding == "p010" ? Format::P010 : Format::NV12;
    allocation->view.width = msg.width;
    allocation->view.height = msg.height;
    allocation->view.num_planes = 1;
    allocation->view.planes[0].size = allocation->size;
    auto buffer = Buffer::create(allocation);
    timeval timestamp = {};
    timestamp.tv_sec = msg.header.stamp.sec;
    timestamp.tv_usec = msg.header.stamp.nanosec / 1000L;
    buffer->setTimestamp(timestamp);
    buffer->bytesused = allocation->size;
    encoder_->queueBuffer(buffer);
    RCLCPP_DEBUG(this->get_logger(),
        "[%s]: transport::type::Image, fd %d, size %zd, flags %x, format %x, width %d, height %d",
        __func__, allocation->fd, allocation->size, allocation->flags, allocation->view.pixelfmt,
        allocation->view.width, allocation->view.height);
    return true;
  }

  virtual bool handle_message_(const sensor_msgs::msg::Image & msg)
  {
    RCLCPP_ERROR(this->get_logger(), "[%s]: sensor_msgs::msg::Image not supported yet ", __func__);
    return false;
  }

  template <typename T, std::enable_if_t<std::is_same_v<transport::type::Image, T>, bool> = true>
  bool publish_message_(std::shared_ptr<Buffer> item)
  {
    auto destroy_callback = [buf = item](std::shared_ptr<lib_mem_dmabuf::DmaBuffer> item) {
      // in case of destroying DmaBuffer, VideoCodec buffer is returned.
    };
    auto msg = std::make_unique<transport::type::Image>();
    msg->header.stamp.sec = item->timestamp.tv_sec;
    msg->header.stamp.nanosec = item->timestamp.tv_usec * 1000L;
    msg->header.frame_id = std::to_string(this->seqno) + "@" + this->framerate;
    auto view = item->view();
    msg->width = view.width;
    msg->height = view.height;
    auto fd = item->fd();
    size_t image_size = item->size();
    msg->dmabuf = std::shared_ptr<lib_mem_dmabuf::DmaBuffer>(
        new lib_mem_dmabuf::DmaBuffer(dup(fd), image_size), destroy_callback);
    msg->encoding = this->format;
    this->publisher->publish(std::move(msg));
    RCLCPP_DEBUG(this->get_logger(),
        "[%s]: transport::type::Image, format[%s] size[%zd] ts[%ld.%ld]", __func__,
        this->format.c_str(), image_size, item->timestamp.tv_sec, item->timestamp.tv_usec);
    return true;
  }

  template <typename T,
      std::enable_if_t<std::is_same_v<sensor_msgs::msg::CompressedImage, T>, bool> = true>
  bool publish_message_(std::shared_ptr<Buffer> item)
  {
    auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
    msg->header.stamp.sec = item->timestamp.tv_sec;
    msg->header.stamp.nanosec = item->timestamp.tv_usec * 1000L;
    // set frame with string "seqno@fps"
    msg->header.frame_id = std::to_string(this->seqno) + "@" + this->framerate;
    item->memory->map();
    size_t image_size = item->bytesused;
    if (not msg->data.empty())
      msg->data.clear();
    sensor_msgs::msg::CompressedImage::_data_type cache;
    cache.insert(cache.begin(), static_cast<uint8_t *>(item->data()),
        static_cast<uint8_t *>(item->data()) + image_size);
    msg->data = cache;
    item->memory->unmap();
    msg->format = this->format;
    this->publisher->publish(std::move(msg));
    RCLCPP_DEBUG(this->get_logger(),
        "[%s]: sensor_msgs::msg::CompressedImage, format[%s] size[%zd] ts[%ld.%ld]", __func__,
        this->format.c_str(), image_size, item->timestamp.tv_sec, item->timestamp.tv_usec);
    return true;
  }

private:
  std::string profile;
  std::string level;
  std::string bitrate;
  std::string rate_control;
  std::string width;
  std::string height;
  std::string framerate;
  std::once_flag check_once;

  std::shared_ptr<Client> encoder_;
  std::shared_ptr<VideoCodec::Notifier> cb_;

  void configure_pixel_format() const
  {
    Format format = Format::NV12;
    if (this->pixel_format == "nv12")
      format = Format::NV12;
    else if (this->pixel_format == "p010")
      format = Format::P010;

    Setting s = { .type = Setting::FORMAT, .data = format };
    encoder_->configure(s);
  }

  void configure_resolution() const
  {
    Resolution resolution = {};
    resolution.width = std::strtoul(width.c_str(), nullptr, 10);
    resolution.height = std::strtoul(height.c_str(), nullptr, 10);
    Setting s = { .type = Resolution::type, .data = resolution };
    encoder_->configure(s);
  }

  std::map<std::string, uint32_t> avc_profile_map_ = { { "baseline", Profile::AVC_BASELINE },
    { "constrained_baseline", Profile::AVC_CONSTRAINED_BASELINE }, { "main", Profile::AVC_MAIN },
    { "high", Profile::AVC_HIGH }, { "constrained_high", Profile::AVC_CONSTRAINED_HIGH } };

  std::map<std::string, uint32_t> hevc_profile_map_ = {
    { "main", Profile::HEVC_MAIN },
    { "main10", Profile::HEVC_MAIN10 },
    { "main_still", Profile::HEVC_MAIN_STILL },
  };

  std::map<std::string, std::map<std::string, uint32_t>> profile_map_ = {
    { "h264", avc_profile_map_ },
    { "h265", hevc_profile_map_ },
  };

  void configure_profile() const
  {
    Profile profile = {};
    if (profile_map_.find(this->format) != profile_map_.end()) {
      auto mapping = profile_map_.at(this->format);
      if (mapping.find(this->profile) != mapping.end()) {
        profile.value = mapping.at(this->profile);
      }
    }
    Setting s = { .type = Setting::PROFILE, .data = profile };
    encoder_->configure(s);
  }

  std::map<std::string, uint32_t> avc_level_map_ = {
    { "1.0", Level::AVC_1_0 },
    { "1.b", Level::AVC_1_B },
    { "1.1", Level::AVC_1_1 },
    { "1.2", Level::AVC_1_2 },
    { "1.3", Level::AVC_1_3 },
    { "2.0", Level::AVC_2_0 },
    { "2.1", Level::AVC_2_1 },
    { "2.2", Level::AVC_2_2 },
    { "3.0", Level::AVC_3_0 },
    { "3.1", Level::AVC_3_1 },
    { "3.2", Level::AVC_3_2 },
    { "4.0", Level::AVC_4_0 },
    { "4.1", Level::AVC_4_1 },
    { "4.2", Level::AVC_4_2 },
    { "5.0", Level::AVC_5_0 },
    { "5.1", Level::AVC_5_1 },
    { "5.2", Level::AVC_5_2 },
    { "6.0", Level::AVC_6_0 },
    { "6.1", Level::AVC_6_1 },
    { "6.2", Level::AVC_6_2 },
  };

  std::map<std::string, uint32_t> hevc_level_map_ = {
    { "1.0", Level::HEVC_1_0 },
    { "2.0", Level::HEVC_2_0 },
    { "2.1", Level::HEVC_2_1 },
    { "3.0", Level::HEVC_3_0 },
    { "3.1", Level::HEVC_3_1 },
    { "4.0", Level::HEVC_4_0 },
    { "4.1", Level::HEVC_4_1 },
    { "5.0", Level::HEVC_5_0 },
    { "5.1", Level::HEVC_5_1 },
    { "5.2", Level::HEVC_5_2 },
    { "6.0", Level::HEVC_6_0 },
    { "6.1", Level::HEVC_6_1 },
    { "6.2", Level::HEVC_6_2 },
  };

  std::map<std::string, std::map<std::string, uint32_t>> level_map_ = {
    { "h264", avc_level_map_ },
    { "h265", hevc_level_map_ },
  };

  void configure_level() const
  {
    Level level = {};
    if (level_map_.find(this->format) != level_map_.end()) {
      auto mapping = level_map_.at(this->format);
      if (mapping.find(this->level) != mapping.end()) {
        level.value = mapping.at(this->level);
      }
    }
    Setting s = { .type = Setting::LEVEL, .data = level };
    encoder_->configure(s);
  };

  void configure_bitrate() const
  {
    Bitrate bitrate = {};
    bitrate.value = std::strtoul(this->bitrate.c_str(), nullptr, 10);
    bitrate.enable = true;
    if (this->rate_control == "cbr")
      bitrate.mode = Bitrate::CBR;
    else if (this->rate_control == "off")
      bitrate.enable = false;
    else
      bitrate.mode = Bitrate::VBR;
    Setting s{ .type = Setting::BITRATE, .data = bitrate };
    encoder_->configure(s);
  }

  void configure_framerate() const
  {
    Framerate framerate = {};
    framerate.value = std::strtoul(this->framerate.c_str(), nullptr, 10);
    Setting s = { .type = Setting::FRAMERATE, .data = framerate };
    encoder_->configure(s);
  }

  std::array<std::function<void()>, 6> settings = {
    [this] { configure_pixel_format(); },
    [this] { configure_resolution(); },
    [this] { configure_profile(); },
    [this] { configure_level(); },
    [this] { configure_bitrate(); },
    [this] { configure_framerate(); },
  };

  void configure_encoder()
  {
    std::for_each(settings.begin(), settings.end(), [](auto & f) { f(); });
  }
};

using Encoder = VideoEncoder<qrb_ros::transport::type::Image, sensor_msgs::msg::CompressedImage>;

template <typename InputMessageT, typename OutputMessageT = InputMessageT>
class PerfVideoEncoder : public VideoEncoder<InputMessageT, OutputMessageT>, public Stats
{
  using Super = VideoEncoder<InputMessageT, OutputMessageT>;

public:
  explicit PerfVideoEncoder(const rclcpp::NodeOptions & options)
    : Super(options), Stats(this->shared_from_this())
  {
    interval_ = (this->declare_parameter("interval", 2000));
  }

  ~PerfVideoEncoder() override = default;

  bool onBufferAvailable(const std::shared_ptr<Buffer> & item) override
  {
    timeval t = item->timestamp;
    auto size = item->bytesused;
    Super::onBufferAvailable(item);
    queue_fbd(t, size);
    return true;
  }

protected:
  bool handle_message_(const transport::type::Image & msg) override
  {
    queue_etb({ msg.header.stamp.sec, msg.header.stamp.nanosec / 1000 }, msg.dmabuf->size());
    return Super::handle_message_(msg);
  }
};

using PerfEncoder =
    PerfVideoEncoder<qrb_ros::transport::type::Image, sensor_msgs::msg::CompressedImage>;

}  // namespace video
}  // namespace qrb_ros

RCLCPP_COMPONENTS_REGISTER_NODE(qrb_ros::video::Encoder);
RCLCPP_COMPONENTS_REGISTER_NODE(qrb_ros::video::PerfEncoder);