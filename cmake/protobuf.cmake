#[[
author: Kuang Fangjun <csukuangfj at gmail dot com>
date: June 06, 2018
]]

#
# PROTOBUF_INCLUDE_DIRS
# PROTOBUF_LIBRARIES
# PROTOBUF_PROTOC_EXECUTABLE

# refer to
# https://cmake.org/cmake/help/v3.4/module/FindProtobuf.html

find_package(Protobuf REQUIRED)
message(STATUS "PROTOBUF_INCLUDE_DIRS: ${PROTOBUF_INCLUDE_DIRS}")
message(STATUS "PROTOBUF_LIBRARIES: ${PROTOBUF_LIBRARIES}")
message(STATUS "Protobuf_PROTOC_EXECUTABLE: ${PROTOBUF_PROTOC_EXECUTABLE}")

include_directories(${PROTOBUF_INCLUDE_DIRS})
