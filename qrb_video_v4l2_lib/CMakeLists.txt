cmake_minimum_required(VERSION 3.22)
project(qrb_video_v4l2_lib VERSION 0.1.7)

set(CMAKE_CXX_STANDARD 17)
include(GNUInstallDirs)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(include include/utils include/v4l2)

find_package(PkgConfig REQUIRED)
pkg_check_modules(glog REQUIRED libglog)

set(INCLUDE_DIRS ${glog_INCLUDE_DIRS})

include_directories(${INCLUDE_DIRS})
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

file(GLOB SOURCES src/utils/*.cpp src/*.cpp src/v4l2/*.cpp)

add_library(v4l2codecs SHARED ${SOURCES})
set_target_properties(v4l2codecs PROPERTIES VERSION ${PROJECT_VERSION} SOVERSION 1)
target_compile_definitions(v4l2codecs PRIVATE -D__USE_FILE_OFFSET64)
target_link_libraries(v4l2codecs ${glog_LIBRARIES})

install(TARGETS v4l2codecs EXPORT ${PROJECT_NAME}Config
  DESTINATION ${CMAKE_INSTALL_LIBDIR} 
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/qrb_video)

set(HEADERS_INSTALL include/)
install(DIRECTORY ${HEADERS_INSTALL}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/qrb_video)

install(EXPORT ${PROJECT_NAME}Config
  DESTINATION share/${PROJECT_NAME}/cmake)
