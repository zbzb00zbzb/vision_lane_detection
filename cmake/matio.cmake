# For libcaffe.so

# matio_LIBRARY

find_library(
        matio_LIBRARY
        NAMES libmatio.so matio
        PATHS
        $ENV{HOME}/software/matio/lib
)

if(NOT matio_LIBRARY)
    message(FATAL_ERROR "Cannot find the matio library")
endif()

message(STATUS "${matio_LIBRARY}")
get_filename_component(matio_lib_dir ${matio_LIBRARY} DIRECTORY)
message(STATUS "matio lib dir: ${matio_lib_dir}")
include_directories(${matio_lib_dir})



