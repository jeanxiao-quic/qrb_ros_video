/*
**************************************************************************************************
* Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
**************************************************************************************************
*/

#include "GstreamerEncoder.hpp"

#include "Log.hpp"

namespace qrb::video_v4l2
{
std::shared_ptr<Client> GstreamerEncoder::create(const Format & compressed)
{
  auto encoder = std::make_shared<GstreamerEncoder>(compressed);
  auto client = std::make_shared<Client>();
  client->setChannelCB(encoder);
  encoder->setNotifier(client);
  return client;
}

GstCaps * GstreamerEncoder::buildSinkCaps() const
{
  const GstVideoFormat fmt = toGstVideoFormat(settings.pixelFormat);
  if (fmt == GST_VIDEO_FORMAT_UNKNOWN) {
    LOGE("%s: no GStreamer video format for pixel format %d", __PRETTY_FUNCTION__,
        static_cast<int>(settings.pixelFormat));
    return nullptr;
  }

  auto * caps =
      gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING, gst_video_format_to_string(fmt),
          "width", G_TYPE_INT, static_cast<gint>(settings.resolution.width), "height", G_TYPE_INT,
          static_cast<gint>(settings.resolution.height), "framerate", GST_TYPE_FRACTION,
          static_cast<gint>(settings.framerate.value), 1, nullptr);
  if (elementFamily() == ElementFamily::Vidc) {
    // Producer-owned dmabufs go in zero-copy; the feature also makes the plugin treat the
    // input as an external pool.
    gst_caps_set_features_simple(caps, gst_caps_features_new("memory:DMABuf", nullptr));
  }
  return caps;
}

GstCaps * GstreamerEncoder::buildSrcCaps() const
{
  const char * media = toGstMediaType(compressedFormat);
  if (media == nullptr) {
    return nullptr;
  }

  // byte-stream / au is what the ROS layer publishes as CompressedImage, and matches what
  // the existing test sink nodes expect.
  auto * caps = gst_caps_new_simple(media, "stream-format", G_TYPE_STRING, "byte-stream",
      "alignment", G_TYPE_STRING, "au", nullptr);

  // The VIDC src pad template carries no profile/level fields, so adding them would make
  // negotiation fail outright.
  if (elementFamily() == ElementFamily::Vidc) {
    if (settings.profile.has_value() or settings.level.has_value()) {
      LOGE(
          "%s: profile/level are not negotiable on the VIDC encoder, ignored", __PRETTY_FUNCTION__);
    }
    return caps;
  }

  if (settings.profile.has_value()) {
    if (const char * name = profileName(settings.profile->value)) {
      gst_caps_set_simple(caps, "profile", G_TYPE_STRING, name, nullptr);
    } else {
      LOGE("%s: no caps name for profile %u", __PRETTY_FUNCTION__, settings.profile->value);
    }
  }
  if (settings.level.has_value()) {
    if (const char * name = levelName(settings.level->value)) {
      gst_caps_set_simple(caps, "level", G_TYPE_STRING, name, nullptr);
    } else {
      LOGE("%s: no caps name for level %u", __PRETTY_FUNCTION__, settings.level->value);
    }
  }

  return caps;
}

static GstPadProbeReturn queryEncoderAllocation(GstPad * pad, GstPadProbeInfo *, gpointer)
{
  if (auto * caps = gst_pad_get_current_caps(pad)) {
    auto * query = gst_query_new_allocation(caps, TRUE);
    if (not gst_pad_query(pad, query)) {
      LOGE("%s: allocation query to the encoder failed", __func__);
    }
    gst_query_unref(query);
    gst_caps_unref(caps);
  }
  return GST_PAD_PROBE_REMOVE;
}

void GstreamerEncoder::configureCodecElement(GstElement * codec) const
{
  if (elementFamily() == ElementFamily::Vidc) {
    // No io-mode / extra-controls on qvidc; bitrate is a plain property.
    if (settings.bitrate.has_value() && settings.bitrate->enable) {
      g_object_set(codec, "target-bitrate", static_cast<guint>(settings.bitrate->value), nullptr);
      gst_util_set_object_arg(G_OBJECT(codec), "rate-control",
          settings.bitrate->mode == Bitrate::CBR ? "constant" : "VBR-CFR");
    }
    // inline-header repeats SPS/PPS before each IDR so a byte-stream consumer can join
    // mid-stream.
    g_object_set(codec, "use-external-pool", TRUE, "inline-header", TRUE, nullptr);

    if (auto * sinkPad = gst_element_get_static_pad(codec, "sink")) {
      gst_pad_add_probe(
          sinkPad, GST_PAD_PROBE_TYPE_BUFFER, queryEncoderAllocation, nullptr, nullptr);
      gst_object_unref(sinkPad);
    }
    return;
  }

  // dmabuf-import on the output (sink) queue is what makes the input path zero-copy:
  // gst_v4l2_allocator_import_dmabuf binds our fd straight into the V4L2 plane group. There
  // is no silent copy fallback in this mode -- import either succeeds or the flow errors --
  // so a regression here is loud rather than a quiet performance loss.
  gst_util_set_object_arg(G_OBJECT(codec), "output-io-mode", "dmabuf-import");
  // MMAP + EXPBUF on capture, so the encoded frame comes out as an fd the ROS transport can
  // dup() into a message.
  gst_util_set_object_arg(G_OBJECT(codec), "capture-io-mode", "dmabuf");

  // Bitrate and the vendor controls have no GObject properties on v4l2videoenc (it installs
  // only V4L2_STD_OBJECT_PROPS), so they go through extra-controls. That resolves names
  // against the driver's own control names, normalised and truncated at 31 chars.
  auto * controls = gst_structure_new_empty("controls");
  if (settings.bitrate.has_value() && settings.bitrate->enable) {
    gst_structure_set(
        controls, "video_bitrate", G_TYPE_INT, static_cast<gint>(settings.bitrate->value), nullptr);
    // V4L2_MPEG_VIDEO_BITRATE_MODE_VBR = 0, _CBR = 1.
    gst_structure_set(controls, "video_bitrate_mode", G_TYPE_INT,
        settings.bitrate->mode == Bitrate::CBR ? 1 : 0, nullptr);
  }
  // Mirrors V4L2_CID_MPEG_VIDC_VUI_TIMING_INFO on the V4L2 path.
  gst_structure_set(controls, "vui_timing_info", G_TYPE_INT, 1, nullptr);

  g_object_set(codec, "extra-controls", controls, nullptr);
  gst_structure_free(controls);
}
}  // namespace qrb::video_v4l2
