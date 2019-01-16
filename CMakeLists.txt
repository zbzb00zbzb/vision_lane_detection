#[[
author: Kuang Fangjun <csukuangfj at gmail dot com>
date: May 29, 2018
]]

cmake_minimum_required(VERSION 3.0.0)

message(STATUS "start cmake top ")
project(lane_line)

option(ENABLE_GTEST "Enable google test" OFF)
option(ENABLE_CPPLINT "Enable cpplint" OFF)
option(ENABLE_ROS "Enable ROS" ON)

if(NOT ENABLE_ROS)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
endif()

set(MY_TYPE SHARED)

if(ENABLE_ROS)
    #set(CATKIN_ENABLE_TESTING OFF CACHE BOOL "disable ros gtest" FORCE)
    #set(ENABLE_GTEST OFF CACHE BOOL "disable test" FORCE)
    #set(hadmap_msgs_DIR "$ENV{HOME}/ws/devel")
    find_package(catkin REQUIRED COMPONENTS
            cv_bridge
            hadmap_msgs
            roscpp
            )
    message(STATUS "catkin_INCLUDE_DIRS: ${catkin_INCLUDE_DIRS}")
    include_directories(${catkin_INCLUDE_DIRS})
    catkin_package()

    set(MY_TYPE STATIC)
endif()

include(cmake/compiler-flags.cmake)
include(cmake/gflags.cmake)
include(cmake/glog.cmake)
include(cmake/matio.cmake)
include(cmake/opencv.cmake)
include(cmake/protobuf.cmake)
include(cmake/segnet.cmake)
include(cmake/yaml_cpp.cmake)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_subdirectory(proto)

include_directories(3rdparty)

if(ENABLE_CPPLINT)
    include(cmake/lint.cmake)
else()
    message(STATUS "cpplint is disabled")
endif()

set(TEST_DATA_DIR "${PROJECT_SOURCE_DIR}/test/data")
if(NOT IS_DIRECTORY ${TEST_DATA_DIR})
    message(FATAL_ERROR "The test data directory ${TEST_DATA_DIR} does not exist!")
endif()

set(CONFIG_PROTO_FILENAME "${PROJECT_SOURCE_DIR}/config/proto.txt")
if(NOT EXISTS ${CONFIG_PROTO_FILENAME})
    message(FATAL_ERROR "The config file ${CONFIG_PROTO_FILENAME} does not exist!")
endif()

#[[
evaluation proto
]]
set(EVAL_PROTO_FILENAME "${PROJECT_SOURCE_DIR}/config/eval.txt")
if(NOT EXISTS ${EVAL_PROTO_FILENAME})
    message(FATAL_ERROR "The configuration file for evaulation ${EVAL_PROTO_FILENAME} does not exist!")
endif()

configure_file(${PROJECT_SOURCE_DIR}/cmake/template/config.h.in ${PROJECT_BINARY_DIR}/lane_line/config.h @ONLY)
include_directories(${PROJECT_BINARY_DIR})

include_directories(include)

add_subdirectory(src)
add_subdirectory(test)
