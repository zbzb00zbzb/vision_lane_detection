#[[
author: Kuang Fangjun <csukuangfj at gmail dot com>
date: May 30, 2018
]]

#
# GTEST_INCLUDE_DIRS
# GTEST_LIBRARIES
#

find_path(GTEST_INCLUDE_DIRS gtest/gtest.h
        PATHS
        $ENV{HOME}/software/googletest/include
        /usr/include
        NO_DEFAULT_PATH
        DOC "Path to gtest/gtest.h"
        )
if(NOT GTEST_INCLUDE_DIRS)
    message(FATAL_ERROR "Could not find gtest/gtest.h")
endif()

find_library(GTEST_LIBRARIES NAMES gtest
        PATHS
        $ENV{HOME}/software/googletest/lib
        /usr/lib/x86_64-linux-gnu
        NO_DEFAULT_PATH
        DOC "Path to libgtest.so"
        )
if(NOT GTEST_LIBRARIES)
    message(FATAL_ERROR "Could not find libgtest.so or libgtest.a")
endif()

include_directories(BEFORE ${GTEST_INCLUDE_DIRS})

message(STATUS "GTEST_INCLUDE_DIRS: ${GTEST_INCLUDE_DIRS}")
message(STATUS "GTEST_LIBRARIES: ${GTEST_LIBRARIES}")
