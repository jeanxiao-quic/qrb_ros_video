/*
**************************************************************************************************
* Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
**************************************************************************************************
*/

#ifndef QRB_ROS_VIDEO__VIDEO_NODE_HPP_
#define QRB_ROS_VIDEO__VIDEO_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <string>

namespace qrb_ros
{
namespace video
{
template <typename InputMessageT, typename OutputMessageT>
class VideoNode : public rclcpp::Node
{
public:
  VideoNode(const std::string & node_name, const rclcpp::NodeOptions & options)
    : Node(node_name, options)
    , pixel_format(declare_parameter<std::string>("pixel-format", "nv12"))
    , format(declare_parameter<std::string>("format", "h264"))
    , log_level(declare_parameter<std::string>("log-level", "info"))
  {
    RCLCPP_INFO(get_logger(), "VideoNode %s has been started.", node_name.c_str());
    if (!options.use_intra_process_comms()) {
      RCLCPP_WARN(get_logger(), "Recommend to enable use_intra_process_comms to optimize max performance");
    }

    subscription = this->template create_subscription<InputMessageT>(
        "input", 10, [this](const InputMessageT & msg) { handle_message(msg); });

    publisher = this->create_publisher<OutputMessageT>("output", 10);

    auto new_level = rclcpp::Logger::Level::Warn;
    if (log_level == "warn") {
      new_level = rclcpp::Logger::Level::Warn;
    } else if (log_level == "info") {
      new_level = rclcpp::Logger::Level::Info;
    } else if (log_level == "debug") {
      new_level = rclcpp::Logger::Level::Debug;
    } else if (log_level == "error") {
      new_level = rclcpp::Logger::Level::Error;
    }
    this->get_logger().set_level(new_level);
  }

protected:
  std::string pixel_format;
  std::string format;
  std::string log_level;
  std::atomic<uint64_t> seqno = 0;

  typename rclcpp::Subscription<InputMessageT>::SharedPtr subscription;
  typename rclcpp::Publisher<OutputMessageT>::SharedPtr publisher;

  virtual bool handle_message(const InputMessageT & msg) = 0;
};

class Stats
{
public:
  explicit Stats(const std::shared_ptr<rclcpp::Node> & node)
  {
    node_ = node;
    counter_ = { 0, 0 };
    total_latency_ = { 0, 0 };
    total_bitrate_ = { 0, 0 };
    interval_ = 2000;
  }

  virtual ~Stats()
  {
    thread_exit_ = true;
    if (thread_.joinable())
      thread_.join();
  }

protected:
  constexpr static int etb = 0;
  constexpr static int fbd = 1;

  uint64_t timeval_to_uint64(timeval value) { return value.tv_sec * 1000000 + value.tv_usec; }

  void log_statistic(uint64_t elapsed)
  {
    std::unique_lock lock(mutex_);
    float fps[2];
    uint64_t average_latency[2] = { 0, 0 };
    fps[etb] = counter_[etb] / (static_cast<float>(elapsed) / 1000);
    fps[fbd] = counter_[fbd] / (static_cast<float>(elapsed) / 1000);
    average_latency[etb] = total_latency_[etb] / counter_[etb];
    average_latency[fbd] = total_latency_[fbd] / counter_[fbd];

    size_t num[2] = { latency_.size(), latency_.size() };
    uint64_t cycle_latency[2] = { 0, 0 };
    uint64_t cycle_average_latency[2] = { 0, 0 };
    float cycle_fps[2];
    std::vector<uint64_t> to_remove;
    std::for_each(latency_.begin(), latency_.end(), [&](auto & sample) {
      if (sample.second[fbd] == 0) {
        num[fbd]--;
        num[etb]--;
        return;
      }
      uint64_t input_latency = sample.second[etb] - sample.first;
      uint64_t encode_latency = sample.second[fbd] - sample.second[etb];
      if (input_latency > 1000000) {
        RCLCPP_INFO(node_->get_logger(), "DEBUG input latency %lu %lu %lu", input_latency,
            sample.second[etb], sample.first);
      }
      if (encode_latency > 1000000) {
        RCLCPP_INFO(node_->get_logger(), "DEBUG encode latency %lu %lu %lu", encode_latency,
            sample.second[fbd], sample.second[etb]);
      }
      cycle_latency[etb] += input_latency;
      cycle_latency[fbd] += encode_latency;
      to_remove.push_back(sample.first);
    });
    cycle_fps[etb] = num[etb] / (static_cast<float>(interval_) / 1000);
    cycle_fps[fbd] = num[fbd] / (static_cast<float>(interval_) / 1000);
    cycle_average_latency[etb] = cycle_latency[etb] / num[etb];
    cycle_average_latency[fbd] = cycle_latency[fbd] / num[fbd];
    std::for_each(
        to_remove.begin(), to_remove.end(), [&](auto & sample) { latency_.erase(sample); });
    uint64_t cycle_bitrate[2] = { 0, 0 };
    uint64_t cycle_byte_size[2] = { 0, 0 };
    uint64_t bitrate[2] = { 0, 0 };
    for (size_t i = 0; i < 2; i++) {
      std::for_each(byte_size_[i].begin(), byte_size_[i].end(),
          [&](auto & sample) { cycle_byte_size[i] += sample; });
      cycle_bitrate[i] = cycle_byte_size[i] * 1000 / interval_;
      byte_size_[i].clear();
      bitrate[i] = (total_bitrate_[i] * (elapsed / interval_ - 1) + cycle_bitrate[i]) /
                   (elapsed / interval_);
      total_bitrate_[i] = bitrate[i];
    }

    lock.unlock();
    RCLCPP_INFO(node_->get_logger(),
        "PERF: etb (fps: %.2f/%.2f, latency %lu.%lums/%lu.%lums, bitrate %luKBps/%luKBps cnt %lu) "
        "fbd "
        "(fps: %.2f/%.2f, "
        "latency "
        "%lu.%lums/%lu.%lums, bitrate %luKBps/%luKBps, cnt %lu)",
        cycle_fps[etb], fps[etb], cycle_average_latency[etb] / 1000,
        cycle_average_latency[etb] % 1000, average_latency[etb] / 1000, average_latency[etb] % 1000,
        cycle_bitrate[etb] / 1000, bitrate[etb] / 1000, counter_[etb], cycle_fps[fbd], fps[fbd],
        cycle_average_latency[fbd] / 1000, cycle_average_latency[fbd] % 1000,
        average_latency[fbd] / 1000, average_latency[fbd] % 1000, cycle_bitrate[fbd] / 1000,
        bitrate[fbd] / 1000, counter_[fbd]);
  }

  void queue_etb(timeval timestamp, size_t size = 0)
  {
    if (not thread_.joinable()) {
      thread_ = std::thread([&]() {
        uint64_t count = 0;
        while (true) {
          std::this_thread::sleep_for(std::chrono::milliseconds(interval_));
          count++;
          log_statistic(count * interval_);
          if (thread_exit_) {
            break;
          }
        }
      });
    }
    std::lock_guard lock(mutex_);
    auto now = get_now();
    uint64_t ts = timeval_to_uint64(timestamp);
    total_latency_[etb] += now - ts;
    latency_[ts] = { now, 0 };
    counter_[etb] += 1;
    byte_size_[etb].push_back(size);
  }

  void queue_fbd(timeval timestamp, size_t size = 0)
  {
    std::lock_guard lock(mutex_);
    auto now = get_now();
    uint64_t ts = timeval_to_uint64(timestamp);
    if (latency_.find(ts) != latency_.end()) {
      total_latency_[fbd] += now - latency_[ts][etb];
      latency_[ts][fbd] = now;
      counter_[fbd] += 1;
      byte_size_[fbd].push_back(size);
    } else {
      int64_t min_diff = INT64_MAX;
      uint64_t closest = ts;
      std::for_each(latency_.begin(), latency_.end(), [&](auto & sample) {
        int64_t sample_diff = ts < sample.first ? sample.first - ts : ts - sample.first;
        if (min_diff > sample_diff) {
          min_diff = sample_diff;
          closest = sample.first;
        }
      });
      RCLCPP_INFO(node_->get_logger(), "fbd unmatched ts %lu, closest ts %lu", ts, closest);
    }
  }

  [[nodiscard]] uint64_t get_now() const
  {
    auto time = node_->now();
    return time.nanoseconds() / 1000;
  }

  uint32_t interval_;

private:
  std::map<uint64_t, std::array<uint64_t, 2>> latency_;
  std::array<uint64_t, 2> total_latency_;
  std::array<uint64_t, 2> counter_;
  std::array<std::vector<uint64_t>, 2> byte_size_;
  std::array<uint64_t, 2> total_bitrate_;
  std::mutex mutex_;
  std::thread thread_;
  bool thread_exit_ = false;
  std::shared_ptr<rclcpp::Node> node_;
};
}  // namespace video
}  // namespace qrb_ros
#endif  // QRB_ROS_VIDEO__VIDEO_NODE_HPP_
