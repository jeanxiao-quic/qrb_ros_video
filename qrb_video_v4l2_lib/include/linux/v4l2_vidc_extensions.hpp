/*
**************************************************************************************************
* Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
**************************************************************************************************
*/

#include <linux/v4l2-controls.h>

#ifndef V4L2_VIDC_EXTENSIONS_HPP_
#define V4L2_VIDC_EXTENSIONS_HPP_

#ifndef V4L2_PIX_FMT_NV12C
#define V4L2_PIX_FMT_NV12C v4l2_fourcc('Q', '0', '8', 'C')
#endif

#ifndef V4L2_PIX_FMT_TP10C
#define V4L2_PIX_FMT_TP10C v4l2_fourcc('Q', '1', '0', 'C')
#endif

#ifndef V4L2_PIX_FMT_P010
#define V4L2_PIX_FMT_P010 v4l2_fourcc('P', '0', '1', '0')
#endif

#ifndef V4L2_CID_MPEG_VIDEO_INTRA_REFRESH_PERIOD_TYPE
#define V4L2_CID_MPEG_VIDEO_INTRA_REFRESH_PERIOD_TYPE (V4L2_CID_CODEC_BASE + 237)
#endif

#ifndef V4L2_BUF_FLAG_CODECCONFIG
#define V4L2_BUF_FLAG_CODECCONFIG 0x01000000
#endif

#ifdef V4L2_CTRL_CLASS_CODEC
#define V4L2_CID_MPEG_VIDC_BASE (V4L2_CTRL_CLASS_CODEC | 0x2000)
#else
#define V4L2_CID_MPEG_VIDC_BASE (V4L2_CTRL_CLASS_MPEG | 0x2000)
#endif

#ifndef V4L2_CID_MPEG_VIDC_VUI_TIMING_INFO
#define V4L2_CID_MPEG_VIDC_VUI_TIMING_INFO                                    \
    (V4L2_CID_MPEG_VIDC_BASE + 0x43)
#endif

#endif  // V4L2_VIDC_EXTENSIONS_HPP_
