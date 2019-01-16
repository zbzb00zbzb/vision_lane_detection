#[[
author: Kuang Fangjun <csukuangfj at gmail dot com>
date: May 30, 2018
]]

#
# OpenCV_INCLUDE_DIRS
# OpenCV_LIBS
#

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV libs: ${OpenCV_LIBS}")