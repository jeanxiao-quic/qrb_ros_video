/*
**************************************************************************************************
* Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
**************************************************************************************************
*/

#include <gst/app/gstappsink.h>
#include <gst/gst.h>

#include <chrono>
#include <functional>
#include <memory>
#include <regex>
#include <string>

#include "base_node.hpp"
#include "qrb_ros_transport_image_type/image.hpp"
#include "qrb_ros_transport_image_type/image_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

namespace qrb_ros
{
namespace video
{

// Forward declarations of specialized classes for registration
class CompressedReader;
class ImageReader;

/**
 * @brief Base source node template class for different message types
 * @tparam MessageT The message type to publish
 */
template <typename MessageT>
class SourceNodeBase : public BaseNode
{
public:
  explicit SourceNodeBase(const std::string & node_name, const rclcpp::NodeOptions & options)
    : BaseNode(node_name, options), duration_(this->declare_parameter("duration", "0"))

  {
    // Create publisher for output data - specific type will be set by derived
    // class
    publisher_ = this->create_publisher<MessageT>("output", 10);

    // Create GStreamer pipeline based on mode
    setup_pipeline();

    parse_duration();

    end_time_ = std::chrono::steady_clock::now() + std::chrono::seconds(duration_seconds_);

    // Create timer to publish data at 1/fps interval
    timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000 / fps_)),
        std::bind(&SourceNodeBase::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "live source at fps %f", fps_);
  }

  ~SourceNodeBase()
  {
    if (!timer_->is_canceled()) {
      timer_ = nullptr;
      available_sample = 0;
    }
    // Base class will clean up GStreamer pipeline
    if (pipeline_) {
      // Send an EOS event to the pipeline to signal elements to finish gracefully.
      // This event flows from the beginning of the pipeline.
      if (!gst_element_send_event(pipeline_, gst_event_new_eos())) {
        RCLCPP_WARN(this->get_logger(), "Failed to send EOS event to the pipeline.");
      } else {
        // Optionally, wait for the EOS message on the bus to confirm propagation.
        // This gives elements time to process the EOS before we force NULL state.
        GstBus * bus = gst_element_get_bus(pipeline_);
        if (bus) {
          RCLCPP_INFO(this->get_logger(),
              "Waiting for EOS message on bus after sending event (duration stop)...");
          GstMessage * msg = gst_bus_timed_pop_filtered(bus,
              2 * GST_SECOND,  // Timeout for waiting (e.g., 2 seconds)
              static_cast<GstMessageType>(GST_MESSAGE_EOS | GST_MESSAGE_ERROR));
          if (msg) {
            if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_EOS) {
              RCLCPP_INFO(this->get_logger(), "EOS message received on bus (duration stop).");
            } else if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
              GError * err = nullptr;
              gchar * debug_info = nullptr;
              gst_message_parse_error(msg, &err, &debug_info);
              RCLCPP_ERROR(this->get_logger(),
                  "Error message received on bus during EOS: %s. Debug: %s",
                  err ? err->message : "Unknown error", debug_info ? debug_info : "None");
              if (err)
                g_error_free(err);
              if (debug_info)
                g_free(debug_info);
            }
            // Unref the message to free resources
            gst_message_unref(msg);
          } else {
            RCLCPP_WARN(this->get_logger(),
                "Timeout waiting for EOS on bus (duration stop). Pipeline might not have fully "
                "processed EOS event.");
          }
          gst_object_unref(bus);
        } else {
          RCLCPP_WARN(
              this->get_logger(), "Could not get pipeline bus to wait for EOS (duration stop).");
        }
      }
      // Set pipeline to NULL state to stop and release all resources.
      gst_element_set_state(pipeline_, GST_STATE_NULL);
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
    }
    if (app_sink_) {
      gst_object_unref(app_sink_);
      app_sink_ = nullptr;
    }
    RCLCPP_WARN(this->get_logger(), "SourceNodeBase destructor finished.");
  }

protected:
  // Method to fill specific message type - to be implemented by derived classes
  virtual void fill_message(typename std::unique_ptr<MessageT> & msg,
      const uint8_t * data,
      size_t size) = 0;

  void parse_duration()
  {
    // Parse the duration string to seconds
    if (duration_.empty() || duration_ == "0") {
      duration_seconds_ = 0;
      return;
    }

    try {
      // Parse human-readable duration string
      std::regex regex(R"((\d+)([smhd]))");
      std::smatch match;
      std::string str = duration_;
      duration_seconds_ = 0;
      while (std::regex_search(str, match, regex)) {
        int value = std::stoi(match[1]);
        char unit = match[2].str()[0];
        switch (unit) {
          case 's':
            duration_seconds_ += value;
            break;
          case 'm':
            duration_seconds_ += value * 60;
            break;
          case 'h':
            duration_seconds_ += value * 3600;
            break;
          case 'd':
            duration_seconds_ += value * 86400;
            break;
          default:
            throw std::invalid_argument("Invalid duration unit");
        }
        str = match.suffix().str();
      }
    } catch (const std::invalid_argument & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse duration: %s", e.what());
      duration_seconds_ = 0;
    }
  }

  // Setup the GStreamer pipeline
  const std::map<std::string, std::function<std::string()>> pipeline_template = {
    { "nv12",
        [this]() {
          std::ostringstream oss;
          oss << "filesrc location=" << url_ << " ! rawvideoparse width=" << width_
              << " height=" << height_ << " format=nv12"
              << " framerate=" << framerate_ << " ! appsink name=sink";
          return oss.str();
        } },
    { "mp4",
        [this]() {
          std::ostringstream oss;
          oss << "filesrc location=" << url_ << " ! qtdemux"
              << " ! " << pixel_format_ << "parse ! video/x-" << pixel_format_
              << ",stream-format=byte-stream,alignment=au"
              << " ! appsink name=sink";
          return oss.str();
        } },
    { "h264",
        [this]() {
          std::ostringstream oss;
          oss << "filesrc location=" << url_
              << " ! h264parse ! video/x-h264,stream-format=byte-stream,alignment=au ! appsink "
                 "name=sink";
          return oss.str();
        } },
    { "h265",
        [this]() {
          std::ostringstream oss;
          oss << "filesrc location=" << url_
              << " ! h265parse ! video/x-h265,stream-format=byte-stream,alignment=au ! appsink "
                 "name=sink";
          return oss.str();
        } },
  };

  void setup_pipeline() override
  {
    std::string pipeline_str;
    try {
      discover_pipeline();
      pipeline_str = pipeline_template.at(format_)();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
          this->get_logger(), "Invalid format: %s. Use 'encoder' or 'decoder'", format_.c_str());
      return;
    }

    if (!create_pipeline(pipeline_str)) {
      return;
    }

    // Get the appsink element
    GstElement * sink = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
    if (!sink) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get appsink element");
      return;
    }

    // Configure appsink
    app_sink_ = GST_APP_SINK(sink);
    gst_app_sink_set_emit_signals(app_sink_, TRUE);
    gst_app_sink_set_drop(app_sink_, FALSE);
    gst_app_sink_set_max_buffers(app_sink_, 3);
    gst_app_sink_set_wait_on_eos(app_sink_, FALSE);

    // Connect the new-sample signal
    g_signal_connect(app_sink_, "new-sample", G_CALLBACK(on_new_sample_static), this);

    // Start the pipeline
    GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
      RCLCPP_ERROR(this->get_logger(), "Failed to start pipeline");
    }
  }

  static GstFlowReturn on_new_sample_static(GstAppSink * sink, gpointer user_data)
  {
    return static_cast<SourceNodeBase *>(user_data)->on_new_sample(sink);
  }

  GstFlowReturn on_new_sample(GstAppSink * sink)
  {
    // just keep the count of available samples
    available_sample++;

    return GST_FLOW_OK;
  }

  void timer_callback()
  {
    static int count = 0;  // Persistent frame counter

    if (app_sink_ && available_sample > 0) {
      if (timer_->is_canceled()) {
        RCLCPP_ERROR(this->get_logger(), "Timer is canceled, stopping further processing.");
        return;
      }
      GstSample * sample = gst_app_sink_try_pull_sample(app_sink_, 5 * GST_MSECOND);
      if (sample) {
        count++;
        GstBuffer * buffer = gst_sample_get_buffer(sample);
        GstMapInfo map;

        auto msg = std::make_unique<MessageT>();
        // Fill message header
        auto ros_now = this->now();  // Use rclcpp::Node::now() for ROS timestamp
        msg->header.stamp = ros_now;
        msg->header.frame_id = std::to_string(count) + "@" + framerate_;

        if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
          fill_message(msg, map.data, map.size);
          gst_buffer_unmap(buffer, &map);
        } else {
          RCLCPP_WARN(this->get_logger(), "Failed to map GStreamer buffer for sample processing.");
        }

        publisher_->publish(std::move(msg));
        gst_sample_unref(sample);
        available_sample--;  // Decrement only after successfully processing a sample
      } else {
        // This can happen if EOS was signaled right after on_new_sample incremented
        // available_sample but before this timer callback could pull it.
        RCLCPP_DEBUG(this->get_logger(),
            "available_sample > 0 but gst_app_sink_pull_sample returned NULL. EOS might be "
            "imminent.");
      }
    }

    // Check for pipeline stop or loop conditions
    auto now_steady = std::chrono::steady_clock::now();

    // Condition 1: Duration for playback has been reached
    if (duration_seconds_ > 0 && now_steady >= end_time_) {
      RCLCPP_INFO(
          this->get_logger(), "Duration reached. Sending EOS event and stopping the pipeline.");
      if (timer_) {
        timer_->cancel();
      }
      return;  // Stop further processing in this callback
    }

    // Condition 2: Natural End-Of-Stream from the GStreamer source
    // This check is relevant for both play-once (duration_seconds_ == 0)
    // and looping (duration_seconds_ > 0 but end_time_ not yet reached).
    if (app_sink_ && gst_app_sink_is_eos(app_sink_)) {
      // Ensure all samples signaled by on_new_sample have been processed before acting on EOS.
      if (available_sample == 0) {
        if (duration_seconds_ == 0) {
          // Play once mode: Natural EOS reached, stop the pipeline.
          RCLCPP_INFO(
              this->get_logger(), "End of stream reached (play once mode). Stopping the pipeline.");
          if (timer_) {
            timer_->cancel();
          }
          // No need to send another EOS event; the source already signaled it.
          return;  // Stop further processing
        } else {
          // Looping mode (duration_seconds_ > 0 and end_time_ not yet reached):
          // Natural EOS reached, so seek to the beginning to loop the content.
          RCLCPP_INFO(this->get_logger(),
              "End of stream reached (looping mode). Seeking to the beginning.");
          if (pipeline_) {
            if (!gst_element_seek_simple(pipeline_, GST_FORMAT_TIME,
                    static_cast<GstSeekFlags>(GST_SEEK_FLAG_FLUSH | GST_SEEK_FLAG_ACCURATE), 0)) {
              RCLCPP_ERROR(this->get_logger(),
                  "Failed to seek to beginning for looping. Stopping pipeline.");
              if (timer_)
                timer_->cancel();  // Stop trying if seek fails
              gst_element_set_state(pipeline_, GST_STATE_NULL);
              return;  // Stop further processing
            }
            // If seek is successful, the pipeline will restart sending samples from the beginning.
          } else {
            RCLCPP_ERROR(
                this->get_logger(), "Pipeline is null, cannot seek for looping. Stopping timer.");
            if (timer_)
              timer_->cancel();
            return;
          }
        }
      }
    }
  }

protected:
  GstAppSink * app_sink_ = nullptr;

  std::string duration_;
  int duration_seconds_ = 0;
  std::chrono::steady_clock::time_point end_time_;

  typename rclcpp::Publisher<MessageT>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex buffer_mutex_;
  std::atomic<uint32_t> available_sample = 0;
  GstClockTime buffer_timestamp_ = GST_CLOCK_TIME_NONE;
  std::thread main_thread_;
};

/**
 * @brief Source node specialized for CompressedImage messages
 */
class CompressedReader : public SourceNodeBase<sensor_msgs::msg::CompressedImage>
{
public:
  explicit CompressedReader(const rclcpp::NodeOptions & options)
    : SourceNodeBase<sensor_msgs::msg::CompressedImage>("CompressedReader", options)
  {
  }

protected:
  void fill_message(sensor_msgs::msg::CompressedImage::UniquePtr & msg,
      const uint8_t * data,
      size_t size) override
  {
    // Set format based on mode
    msg->format = format_;

    // Copy the data
    msg->data.resize(size);
    memcpy(msg->data.data(), data, size);
  }
};

/**
 * @brief Source node specialized for qrb_ros::transport::type::Image messages
 */
class ImageReader : public SourceNodeBase<qrb_ros::transport::type::Image>
{
public:
  explicit ImageReader(const rclcpp::NodeOptions & options)
    : SourceNodeBase<qrb_ros::transport::type::Image>("ImageReader", options)
  {
  }

protected:
  void fill_message(std::unique_ptr<qrb_ros::transport::type::Image> & msg,
      const uint8_t * data,
      size_t size) override
  {
    // Configure message based on mode
    msg->encoding = format_;
    msg->width = std::stoi(width_);
    msg->height = std::stoi(height_);
    size_t aligned_size = qrb_ros::transport::image_utils::get_image_align_size(
        msg->width, msg->height, msg->encoding);
    msg->dmabuf = lib_mem_dmabuf::DmaBuffer::alloc(aligned_size, "/dev/dma_heap/system");
    if (!msg->dmabuf) {
      RCLCPP_ERROR(this->get_logger(), "Failed to allocate dmabuf");
      return;
    }
    qrb_ros::transport::image_utils::save_image_to_dmabuf(
        msg->dmabuf, data, msg->width, msg->height, msg->width, msg->encoding, true);
    msg->dmabuf->set_auto_release(true);
  }
};

}  // namespace video
}  // namespace qrb_ros

// Register the components with class loader
RCLCPP_COMPONENTS_REGISTER_NODE(qrb_ros::video::CompressedReader)
RCLCPP_COMPONENTS_REGISTER_NODE(qrb_ros::video::ImageReader)