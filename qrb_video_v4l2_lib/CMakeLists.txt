cmake_minimum_required(VERSION 3.22)
project(qrb_video_v4l2_lib VERSION 0.2.0)

set(CMAKE_CXX_STANDARD 17)
include(GNUInstallDirs)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(QRB_VIDEO_ENABLE_V4L2 "Build the direct V4L2 backend" OFF)
option(QRB_VIDEO_ENABLE_GST "Build the GStreamer backend" ON)
set(QRB_VIDEO_DEFAULT_TARGET "gst" CACHE STRING
  "Backend used by Encoder/Decoder::create(mime): v4l2 | gst")
set_property(CACHE QRB_VIDEO_DEFAULT_TARGET PROPERTY STRINGS v4l2 gst)

if(NOT QRB_VIDEO_ENABLE_V4L2 AND NOT QRB_VIDEO_ENABLE_GST)
  message(FATAL_ERROR
    "At least one backend must be enabled "
    "(-DQRB_VIDEO_ENABLE_V4L2=ON and/or -DQRB_VIDEO_ENABLE_GST=ON)")
endif()

if(QRB_VIDEO_DEFAULT_TARGET STREQUAL "v4l2")
  if(NOT QRB_VIDEO_ENABLE_V4L2)
    message(FATAL_ERROR "QRB_VIDEO_DEFAULT_TARGET=v4l2 but QRB_VIDEO_ENABLE_V4L2=OFF")
  endif()
elseif(QRB_VIDEO_DEFAULT_TARGET STREQUAL "gst")
  if(NOT QRB_VIDEO_ENABLE_GST)
    message(FATAL_ERROR "QRB_VIDEO_DEFAULT_TARGET=gst but QRB_VIDEO_ENABLE_GST=OFF")
  endif()
else()
  message(FATAL_ERROR
    "QRB_VIDEO_DEFAULT_TARGET must be 'v4l2' or 'gst', got '${QRB_VIDEO_DEFAULT_TARGET}'")
endif()

message(STATUS "qrb_video: v4l2 backend ${QRB_VIDEO_ENABLE_V4L2}, "
  "gst backend ${QRB_VIDEO_ENABLE_GST}, default ${QRB_VIDEO_DEFAULT_TARGET}")

include_directories(include include/utils)

find_library(GLOG_LIBRARY NAMES glog REQUIRED)
add_compile_options(
  -Wall
  -Wpedantic
  -Werror
  -Wno-unused-parameter
  -Wno-unused-variable
  -Wno-unused-function
  -Wno-unused-value
  -std=c++17
)

file(GLOB SOURCES src/utils/*.cpp src/*.cpp)

if(QRB_VIDEO_ENABLE_V4L2)
  include_directories(include/v4l2)
  file(GLOB V4L2_SOURCES src/v4l2/*.cpp)
  list(APPEND SOURCES ${V4L2_SOURCES})
endif()

if(QRB_VIDEO_ENABLE_GST)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(GST REQUIRED IMPORTED_TARGET
    gstreamer-1.0
    gstreamer-app-1.0
    gstreamer-video-1.0
    gstreamer-allocators-1.0
  )
  include_directories(include/gst)
  file(GLOB GST_SOURCES src/gst/*.cpp)
  list(APPEND SOURCES ${GST_SOURCES})
endif()

add_library(v4l2codecs SHARED ${SOURCES})
set_target_properties(v4l2codecs PROPERTIES VERSION ${PROJECT_VERSION} SOVERSION 1)
target_compile_definitions(v4l2codecs PRIVATE -D__USE_FILE_OFFSET64 GLOG_USE_GLOG_EXPORT)
target_link_libraries(v4l2codecs PRIVATE ${GLOG_LIBRARY})

# Always defined as 0/1 so both #if and plain C++ expressions can use them. PRIVATE: these
# must never reach an installed header, or consumers would need the same flags to agree.
if(QRB_VIDEO_ENABLE_V4L2)
  target_compile_definitions(v4l2codecs PRIVATE QRB_VIDEO_HAS_V4L2=1)
else()
  target_compile_definitions(v4l2codecs PRIVATE QRB_VIDEO_HAS_V4L2=0)
endif()

if(QRB_VIDEO_ENABLE_GST)
  target_compile_definitions(v4l2codecs PRIVATE QRB_VIDEO_HAS_GST=1)
  # SYSTEM so that GLib/GStreamer headers do not trip -Wpedantic -Werror above.
  target_link_libraries(v4l2codecs PRIVATE PkgConfig::GST)
  target_include_directories(v4l2codecs SYSTEM PRIVATE ${GST_INCLUDE_DIRS})
else()
  target_compile_definitions(v4l2codecs PRIVATE QRB_VIDEO_HAS_GST=0)
endif()

if(QRB_VIDEO_DEFAULT_TARGET STREQUAL "gst")
  target_compile_definitions(v4l2codecs PRIVATE QRB_VIDEO_DEFAULT_TARGET_GST=1)
else()
  target_compile_definitions(v4l2codecs PRIVATE QRB_VIDEO_DEFAULT_TARGET_GST=0)
endif()

install(TARGETS v4l2codecs EXPORT ${PROJECT_NAME}Config
  DESTINATION ${CMAKE_INSTALL_LIBDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/qrb_video)

# Backend headers install only when that backend is compiled in: shipping
# gst/GstreamerEncoder.hpp without the code behind it gives a consumer that includes it
# link errors, and puts a GStreamer include dependency in a package that has no use for
# one. No header under include/*.hpp includes a backend header, so this is a clean cut.
# include/linux/ holds the vidc extension CIDs and is used only by the V4L2 backend.
install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/qrb_video
  PATTERN "v4l2" EXCLUDE
  PATTERN "linux" EXCLUDE
  PATTERN "gst" EXCLUDE)

if(QRB_VIDEO_ENABLE_V4L2)
  install(DIRECTORY include/v4l2 include/linux
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/qrb_video)
endif()

if(QRB_VIDEO_ENABLE_GST)
  install(DIRECTORY include/gst
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/qrb_video)
endif()

install(EXPORT ${PROJECT_NAME}Config
  DESTINATION share/${PROJECT_NAME}/cmake)
