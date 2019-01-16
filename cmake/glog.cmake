#[[
author: Kuang Fangjun <csukuangfj at gmail dot com>
date: May 30, 2018
]]

#
# GLOG_INCLUDE_DIRS
# GLOG_LIBRARIES
#

find_path(GLOG_INCLUDE_DIRS glog/logging.h PATHS /usr/include DOC "Path to glog/logging.h")
if(NOT GLOG_INCLUDE_DIRS)
    message(FATAL_ERROR "Could not find glog/logging.h")
endif()

find_library(GLOG_LIBRARIES glog PATHS /usr/lib/x86_64-linux-gnu)
if(NOT GLOG_LIBRARIES)
    message(FATAL_ERROR "Could not find libglog.so")
endif()

include_directories(${GLOG_INCLUDE_DIRS})

message(STATUS "GLOG_INCLUDE_DIRS: ${GLOG_INCLUDE_DIRS}")
message(STATUS "GLOG_LIBRAIES: ${GLOG_LIBRARIES}")
