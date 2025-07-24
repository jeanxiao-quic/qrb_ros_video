/*
**************************************************************************************************
* Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
**************************************************************************************************
*/

#include <gst/allocators/gstdmabuf.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <gst/video/gstvideometa.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "base_node.hpp"
#include "qrb_ros_transport_image_type/image.hpp"
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
class CompressedWriter;
class ImageWriter;

/**
 * @brief Base sink node template class for different message types
 * @tparam MessageT The message type to subscribe to
 */
template <typename MessageT>
class SinkNodeBase : public BaseNode
{
public:
  explicit SinkNodeBase(const std::string & node_name, const rclcpp::NodeOptions & options)
    : BaseNode(node_name, options), frame_counter_(0)
  {
    // Create subscription for input data - specific callback will be set by
    // derived class
    setup_subscription();
  }

  ~SinkNodeBase()
  {
    // Send EOS (End of Stream) and wait for pipeline to process it
    if (app_src_ && pipeline_) {  // Ensure both app_src_ and pipeline_ are valid
      GstFlowReturn ret = gst_app_src_end_of_stream(app_src_);
      if (ret != GST_FLOW_OK) {
        RCLCPP_WARN(this->get_logger(), "gst_app_src_end_of_stream() failed with %s",
            gst_flow_get_name(ret));
      } else {
        RCLCPP_INFO(this->get_logger(), "EOS sent to appsrc. Waiting for EOS message on the "
                                        "bus...");

        GstBus * bus = gst_element_get_bus(pipeline_);
        if (bus) {
          // Wait for EOS or ERROR message, with a timeout (e.g., 5 seconds)
          // GST_CLOCK_TIME_NONE can be used for an indefinite wait, but a timeout is safer.
          GstMessage * msg = gst_bus_timed_pop_filtered(bus, 5 * GST_SECOND,
              static_cast<GstMessageType>(GST_MESSAGE_EOS | GST_MESSAGE_ERROR));

          if (msg) {
            if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_EOS) {
              RCLCPP_INFO(this->get_logger(),
                  "EOS message received from pipeline. Stream ended gracefully.");
            } else if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
              GError * err = nullptr;
              gchar * debug_info = nullptr;
              gst_message_parse_error(msg, &err, &debug_info);
              RCLCPP_ERROR(this->get_logger(),
                  "Error message received from pipeline during EOS: %s. Debug: %s",
                  err ? err->message : "Unknown error", debug_info ? debug_info : "None");
              if (err)
                g_error_free(err);
              if (debug_info)
                g_free(debug_info);
            }
            gst_message_unref(msg);
          } else {
            RCLCPP_WARN(this->get_logger(),
                "Timeout waiting for EOS message on the bus. Pipeline might not have processed EOS "
                "completely.");
          }
          gst_object_unref(bus);
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to get pipeline bus to wait for EOS.");
          // Fallback to a simple sleep if bus is not available, though this is less reliable.
          std::this_thread::sleep_for(std::chrono::seconds(1));
        }
      }
      gst_element_set_state(pipeline_, GST_STATE_NULL);
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
    } else {
      if (!app_src_)
        RCLCPP_WARN(this->get_logger(), "app_src_ is null in destructor, cannot send EOS.");
      if (!pipeline_)
        RCLCPP_WARN(
            this->get_logger(), "pipeline_ is null in destructor, cannot wait for EOS on bus.");
    }

    // Base class (BaseNode) destructor will be called next and should handle
    // setting the pipeline to GST_STATE_NULL and unreffing it.
    // gst_object_unref(app_src_);
    // app_src_ = nullptr;
    RCLCPP_WARN(this->get_logger(), "SinkNodeBase destructor finished.");
  }

protected:
  // Setup the subscription for the specific message type
  virtual void setup_subscription()
  {
    subscription_ = this->create_subscription<MessageT>(
        "input", 10, std::bind(&SinkNodeBase::handle_message, this, std::placeholders::_1));
  }

  virtual void handle_message(const std::shared_ptr<MessageT> msg) { this->process_message(msg); }

  // Method to extract data from specific message type - to be implemented by
  // derived classes
  virtual GstBuffer * extract_data(const std::shared_ptr<MessageT> & msg) = 0;

  void process_message(const std::shared_ptr<MessageT> & msg)
  {
    // Extract data from the message
    GstBuffer * buffer = extract_data(msg);
    if (!buffer) {
      RCLCPP_ERROR(this->get_logger(), "Failed to extract data from message");
      return;
    }

    RCLCPP_DEBUG(this->get_logger(), "Received frame of size: %zu bytes with timestamp %ld",
        gst_buffer_get_size(buffer), GST_BUFFER_PTS(buffer));

    setup_pipeline();

    // Push the buffer into appsrc
    GstFlowReturn ret = gst_app_src_push_buffer(app_src_, buffer);
    if (ret != GST_FLOW_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to push buffer to GStreamer pipeline: %d", ret);
    } else {
      frame_counter_++;
      RCLCPP_DEBUG(this->get_logger(), "Pushed frame %ld to pipeline", frame_counter_);
    }
  }

  const std::map<std::string, std::function<std::string()>> pipeline_template = {
    { "mp4",
        [this]() {
          // Efficient string construction using std::ostringstream
          std::ostringstream oss;
          oss << "appsrc name=src ! " << pixel_format_
              << "parse ! qtmux ! filesink sync=false location=" << url_;

          return oss.str();
        } },
    { "raw",
        [this]() {
          std::ostringstream oss;
          oss << "appsrc name=src ! filesink sync=false "
                 "location="
              << url_;
          return oss.str();
        } },
  };

  void setup_pipeline() override
  {
    if (pipeline_ != nullptr) {
      return;
    }
    std::string pipeline_str;
    try {
      pipeline_str = pipeline_template.at(format_)();
    } catch (const std::out_of_range & e) {
      if (format_ == "h264" || format_ == "h265" || format_ == "nv12") {
        pipeline_str = pipeline_template.at("raw")();
      } else {
        RCLCPP_ERROR(this->get_logger(), "Invalid format: %s. Use 'mp4' or 'raw'", format_.c_str());
        return;
      }
    }

    if (!create_pipeline(pipeline_str)) {
      return;
    }

    // Get the appsrc element
    GstElement * src = gst_bin_get_by_name(GST_BIN(pipeline_), "src");
    if (!src) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get appsrc element");
      return;
    }

    // Configure appsrc
    app_src_ = GST_APP_SRC(src);
    gst_app_src_set_stream_type(app_src_, GST_APP_STREAM_TYPE_STREAM);
    gst_app_src_set_emit_signals(app_src_, TRUE);
    gst_app_src_set_max_bytes(app_src_, 0);  // No limit on buffer size
    g_object_set(G_OBJECT(app_src_), "is-live", TRUE, NULL);
    g_object_set(G_OBJECT(app_src_), "do-timestamp", FALSE, NULL);

    // Set caps based on mode
    GstCaps * caps;
    std::map<std::string, std::string> mime_mapping = { { "h264", "video/x-h264" },
      { "h265", "video/x-h265" } };
    if (pixel_format_ == "nv12") {
      // For nv12 encoding input (raw video)
      caps = gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING, "NV12", "width", G_TYPE_INT,
          1920, "height", G_TYPE_INT, 1080, "framerate", GST_TYPE_FRACTION, static_cast<guint>(fps_), 1, NULL);
    } else {
      // For decoder input (h264)
      try {
        caps = gst_caps_new_simple(mime_mapping.at(pixel_format_).c_str(), "stream-format",
            G_TYPE_STRING, "byte-stream", "alignment", G_TYPE_STRING, "au", "framerate", GST_TYPE_FRACTION, static_cast<guint>(fps_), 1, NULL);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Invalid pixel format: %s", pixel_format_.c_str());
        return;
      }
    }

    gst_app_src_set_caps(app_src_, caps);
    gst_caps_unref(caps);

    // Connect need-data signal
    g_signal_connect(app_src_, "need-data", G_CALLBACK(on_need_data_static), this);

    // Start the pipeline
    GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
      RCLCPP_ERROR(this->get_logger(), "Failed to start pipeline");
    }
  }

  static void on_need_data_static(GstAppSrc * src, guint length, gpointer user_data)
  {
    static_cast<SinkNodeBase *>(user_data)->on_need_data(src, length);
  }

  void on_need_data(GstAppSrc * src, guint length)
  {
    // This function is called when the appsrc element needs more data
    // We don't need to do anything here, as we push data when we receive it via
    // ROS messages
    RCLCPP_INFO(this->get_logger(), "Src: %s, Need %d bytes", gst_element_get_name(src), length);
  }

  uint64_t frame_counter_;
  typename rclcpp::Subscription<MessageT>::SharedPtr subscription_;

  GstAppSrc * app_src_ = nullptr;
};

/**
 * @brief Sink node specialized for CompressedImage messages
 */
class CompressedWriter : public SinkNodeBase<sensor_msgs::msg::CompressedImage>
{
public:
  explicit CompressedWriter(const rclcpp::NodeOptions & options)
    : SinkNodeBase<sensor_msgs::msg::CompressedImage>("compressed_writer", options)
  {
  }

protected:
  struct ReleaseData
  {
    std::shared_ptr<sensor_msgs::msg::CompressedImage> data;
  };

  GstBuffer * extract_data(const std::shared_ptr<sensor_msgs::msg::CompressedImage> & msg) override
  {
    auto & data = msg->data;
    auto release_data = new ReleaseData{ msg };

    if (pixel_format_.empty()) {
      pixel_format_ = msg->format;
      RCLCPP_INFO(this->get_logger(), "Detected pixel format: %s", pixel_format_.c_str());

      auto frame_id = msg->header.frame_id;
      if (! frame_id.empty()) {
        // extract framerate with format seqno@fps
        auto pos = frame_id.find('@');
        if (pos != std::string::npos) {
          auto fps_str = frame_id.substr(pos + 1);
          try {
            float fps = std::stof(fps_str);
            RCLCPP_INFO(this->get_logger(), "Detected framerate: %.2f", fps);
            if (fps_ != fps) {
              fps_ = fps;
              RCLCPP_INFO(this->get_logger(), "Updated framerate to: %.2f", fps_);
            }
          } catch (const std::invalid_argument & e) {
            RCLCPP_ERROR(this->get_logger(), "Invalid framerate format: %s", fps_str.c_str());
          }
        }
      }
    }

    // Create a new buffer that wraps the vector's data
    GstBuffer * buffer = gst_buffer_new_wrapped_full(GST_MEMORY_FLAG_READONLY, data.data(),
        data.size(), 0, data.size(), release_data, [](gpointer data) {
          if (data != nullptr) {
            auto release_data = static_cast<ReleaseData *>(data);
            delete release_data;
          }
        });

    if (!buffer) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create buffer");
      delete release_data;
      return nullptr;
    }

    auto stamp = msg->header.stamp;
    GST_BUFFER_PTS(buffer) = stamp.sec * GST_SECOND + stamp.nanosec;
    GST_BUFFER_DTS(buffer) = GST_BUFFER_PTS(buffer);

    return buffer;
  }

private:
};

/**
 * @brief Sink node specialized for qrb_ros::transport::type::Image messages
 */
class ImageWriter : public SinkNodeBase<qrb_ros::transport::type::Image>
{
public:
  explicit ImageWriter(const rclcpp::NodeOptions & options)
    : SinkNodeBase<qrb_ros::transport::type::Image>("image_writer", options)
  {
  }

protected:
  struct ReleaseData
  {
    std::shared_ptr<qrb_ros::transport::type::Image> data;
  };

  GstBuffer * extract_data(const std::shared_ptr<qrb_ros::transport::type::Image> & msg) override
  {
    std::map<std::string, GstVideoFormat> format_map = {
      { "nv12", GST_VIDEO_FORMAT_NV12 },
      { "i420", GST_VIDEO_FORMAT_I420 },
    };

    if (pixel_format_.empty()) {
      pixel_format_ = msg->encoding;
      RCLCPP_INFO(this->get_logger(), "Detected pixel format: %s", pixel_format_.c_str());
    }

    // Create a buffer from the dmabuf
    GstAllocator * allocator = gst_dmabuf_allocator_new();
    GstMemory * memory =
        gst_dmabuf_allocator_alloc(allocator, msg->dmabuf->fd(), msg->dmabuf->size());

    if (memory == nullptr) {
      RCLCPP_ERROR(this->get_logger(), "Failed to allocate memory");
      return nullptr;
    }

    auto release_data = new ReleaseData{ msg };
    gst_mini_object_set_qdata(GST_MINI_OBJECT(memory),
        g_quark_from_string("qrb_ros::transport::type::Image"), release_data, [](gpointer data) {
          auto release_data = static_cast<ReleaseData *>(data);
          delete release_data;
        });

    GstBuffer * buffer = gst_buffer_new();
    gst_buffer_append_memory(buffer, memory);

    auto stamp = msg->header.stamp;
    auto width = msg->width;
    auto height = msg->height;
    GstVideoFormat format = GST_VIDEO_FORMAT_UNKNOWN;
    try {
      format = format_map.at(msg->encoding);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Invalid encoding: %s", msg->encoding.c_str());
      return nullptr;
    }
    GST_BUFFER_PTS(buffer) = stamp.sec * GST_SECOND + stamp.nanosec;
    GST_BUFFER_DTS(buffer) = GST_BUFFER_PTS(buffer);
    gst_buffer_add_video_meta(buffer, GST_VIDEO_FRAME_FLAG_NONE, format, width, height);

    gst_object_unref(allocator);

    return buffer;
  }

private:
};

}  // namespace video
}  // namespace qrb_ros

// Register the components with class loader
RCLCPP_COMPONENTS_REGISTER_NODE(qrb_ros::video::CompressedWriter)
RCLCPP_COMPONENTS_REGISTER_NODE(qrb_ros::video::ImageWriter)
