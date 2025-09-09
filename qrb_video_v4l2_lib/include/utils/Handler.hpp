/*
 **************************************************************************************************
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 **************************************************************************************************
 */

#ifndef QRB_VIDEO_V4L2__HANDLER_HPP_
#define QRB_VIDEO_V4L2__HANDLER_HPP_

#include <any>
#include <future>

namespace qrb::video_v4l2
{
struct Message;
class Looper;

using value_type = int32_t;

class Handler : public std::enable_shared_from_this<Handler>
{
public:
  Handler();

  virtual ~Handler() = default;

  Handler(const Handler * another);

  enum class Flags
  {
    DEFAULT = 0,
    SYNC = 1,
  };

  virtual bool handleMessage(const std::shared_ptr<Message> & msg) = 0;

  std::shared_ptr<Message> obtainMessage(int what);

  int32_t sendMessage(std::shared_ptr<Message> & msg);

  bool sendMessageAsync(std::shared_ptr<Message> & msg);

protected:
  bool finishMessage(const std::shared_ptr<Message> & msg, value_type ret);

private:
  std::condition_variable cv;
  std::mutex lock;

  std::shared_ptr<Looper> looper;
};

inline std::underlying_type_t<Handler::Flags> operator&(const Handler::Flags a,
    const Handler::Flags b)
{
  return static_cast<std::underlying_type_t<Handler::Flags>>(a) &
         static_cast<std::underlying_type_t<Handler::Flags>>(b);
}

// template<typename Integer>
// std::underlying_type_t<Integer> operator&(
//     std::enable_if_t<std::is_enum_v<Integer>, const Integer> a,
//     std::enable_if_t<std::is_enum_v<Integer>, const Integer> b) {
//     return static_cast<std::underlying_type_t<Integer>>(a) &
//     static_cast<std::underlying_type_t<Integer>>(b);
// }

struct Message
{
  explicit Message(int32_t what) : what(what), flags(Handler::Flags::DEFAULT) {}

  int32_t what;
  Handler::Flags flags;
  std::weak_ptr<Handler> handler;
  std::promise<value_type> promise;
  std::any data;
};
}  // namespace qrb::video_v4l2

#endif  // QRB_VIDEO_V4L2__HANDLER_HPP_
