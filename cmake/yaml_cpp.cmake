#[[
author: Kuang Fangjun <csukuangfj at gmail dot com>
date: June 05, 2018
]]

#
# sudo apt install libyaml-cpp0.5v5 libyaml-cpp-dev
#

#
#  YAML_CPP_INCLUDE_DIR - include directory
#  YAML_CPP_LIBRARIES   - libraries to link against
#
find_package(yaml-cpp REQUIRED)

message(STATUS "YAML_CPP_INCLUDE_DIR: ${YAML_CPP_INCLUDE_DIR}")
message(STATUS "YAML_CPP_LIBRARIES: ${YAML_CPP_LIBRARIES}")

include_directories(${YAML_CPP_INCLUDE_DIR})
