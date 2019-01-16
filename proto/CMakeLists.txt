#[[
author: Kuang Fangjun <csukuangfj at gmail dot com>
date: June 06, 2018
]]

# refer to FindProtobuf.cmake

file(GLOB srcs *.proto)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${srcs})

message(STATUS "proto definitions: ${srcs}")
message(STATUS "PROTO_SRCS: ${PROTO_SRCS}")
message(STATUS "PROTO_HDRS: ${PROTO_HDRS}")

add_library(
        ${PROJECT_NAME}_proto
        ${MY_TYPE}
        ${PROTO_SRCS}
)
target_link_libraries(
        ${PROJECT_NAME}_proto
        ${PROTOBUF_LIBRARIES}
)
