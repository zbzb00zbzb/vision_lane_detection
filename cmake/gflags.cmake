#[[
author: Kuang Fangjun <csukuangfj at gmail dot com>
date: May 30, 2018
]]

#
# GFLAGS_INCLUDE_DIRS
# GFLAGS_LIBRARIES
#

find_path(GFLAGS_INCLUDE_DIRS gflags/gflags.h PATHS /usr/include DOC "Path to gflags/gflags.h")
if(NOT GFLAGS_INCLUDE_DIRS)
    message(FATAL_ERROR "Could not find gflags/gflags.h")
endif()

find_library(GFLAGS_LIBRARIES NAMES libgflags.so gflags PATHS /usr/lib/x86_64-linux-gnu DOC "Path to libgflags.so")
if(NOT GFLAGS_LIBRARIES)
    message(FATAL_ERROR "Could not find libgflags.so")
endif()

include_directories(${GFLAGS_INCLUDE_DIRS})

message(STATUS "GFLAGS_INCLUDE_DIRS: ${GFLAGS_INCLUDE_DIRS}")
message(STATUS "GFLAGS_LIBRARIES: ${GFLAGS_LIBRARIES}")
