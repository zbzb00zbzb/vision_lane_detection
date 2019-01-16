#[[
author: Kuang Fangjun <csukuangfj at gmail dot com>
date: September 05, 2018
]]

set(srcs
        ipm_parameters_compare.cpp
        )
add_executable(ipm-parameters-cnn-compare ${srcs})
target_link_libraries(
        ipm-parameters-cnn-compare
        ${PROJECT_NAME}_core
)


add_executable(extract-ground-truth-single-image-test extract-ground-truth-single-image-test.cpp)
target_link_libraries(extract-ground-truth-single-image-test ${PROJECT_NAME}_core)
