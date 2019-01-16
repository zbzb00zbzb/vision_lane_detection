#[[
author: Kuang Fangjun <csukuangfj at gmail dot com>
date: June 05, 2018
]]

message(STATUS "start cmake caffe ")
# CAFFE_LIBRARY

# model path for segnet
# the directory MUST include
# - the model file: deploy.prototxt
# - the weight file: after_bn_calc2.caffemodel

message(STATUS "Processor: ${CMAKE_SYSTEM_PROCESSOR}")
set(IS_PX2 FALSE)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(IS_PX2 TRUE)
endif()

set(CNN_MODEL_PATH "${PROJECT_SOURCE_DIR}/models")
if(NOT IS_DIRECTORY ${CNN_MODEL_PATH})
    message(FATAL_ERROR "The model directory ${CNN_MODEL_PATH} does not exist.")
endif()

set(CNN_PROTO_FILENAME "${CNN_MODEL_PATH}/deploy.prototxt")
set(CNN_TRAINED_FILENAME "${CNN_MODEL_PATH}/after_bn_calc2.caffemodel")

if(NOT EXISTS ${CNN_PROTO_FILENAME})
    message(FATAL_ERROR "Segnet proto file ${CNN_PROTO_FILENAME} does not exist!")
endif()

if(NOT EXISTS ${CNN_TRAINED_FILENAME})
    message(FATAL_ERROR "Segnet trained file ${CNN_TRAINED_FILENAME} does not exist!")
endif()

message(STATUS "proto file: ${CNN_PROTO_FILENAME}")
message(STATUS "trained file: ${CNN_TRAINED_FILENAME}")

# use GPU for caffe or not
set(CNN_USE_GPU ON)
#set(CNN_USE_GPU OFF)

if(CNN_USE_GPU)
    if(IS_PX2)
        find_library(CAFFE_LIBRARY caffe PATHS ./lib-arm)
    else()
        find_library(CAFFE_LIBRARY caffe PATHS ./lib)
    endif()

    if(NOT CAFFE_LIBRARY)
        message(FATAL_ERROR "Could not find libcaffe.so")
    else()
        message(STATUS "Caffe lib: ${CAFFE_LIBRARY}")
    endif()
    include_directories(/usr/local/cuda-8.0/include)
    include_directories(/usr/local/cuda/include)
else()
    message(FATAL_ERROR "To copy the libcafee cpu library, @fangjun")
    find_library(CAFFE_LIBRARY caffe PATHS ./lib-cpu)
    # for caffe
    add_definitions(-DCPU_ONLY)
endif()

add_definitions(-DUSE_OPENCV)

