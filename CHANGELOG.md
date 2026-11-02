## 0.2.0 (2026-09-21)

- feat(gst): add a GStreamer backend selectable through `VideoCodec::Target`
- feat(lib): report failures through a new `VideoCodec::Notifier::onError` callback
- feat(build): select backends at compile time with `QRB_VIDEO_ENABLE_V4L2` /
  `QRB_VIDEO_ENABLE_GST` / `QRB_VIDEO_DEFAULT_TARGET`
- refactor(lib): move the dma_heap allocator out of the V4L2 backend so both backends share it
- refactor(lib): drop backend headers from `Encoder.hpp` / `Decoder.hpp`
- build(lib): install a backend's headers only when that backend is compiled in
- fix(lib): default-initialise `Buffer`'s members. `Buffer() = default` left `flags`
  indeterminate, so `isEOS()` on a freshly created buffer could spuriously start a drain.
- Note: SOVERSION stays 1. `onError` is source-compatible (non-pure, so existing
  `Notifier` subclasses build unchanged), but it does add a `Notifier` vtable slot, so
  a consumer must be rebuilt against 0.2.0 headers -- do not drop this library under a
  binary built against 0.1.7. The shlibs minimum version (>= 0.2.0) enforces that on the deb path.

## 0.1.7 (2025-09-25)

- docs: update usage  and installation instructions in README.md
- docs: update README.md with new template
- fix(mp4 container): video processing with framerate handling
- test: qrb_ros_video: Add GStreamer-based test nodes
- fix: qrb_ros_video: Improve code quality
- docs: add chapter on commit message format
- Added: enable  actionlin

## 0.1.0 (2025-01-16)

- Initial version release for Humble
- Contributors: Jean Xiao