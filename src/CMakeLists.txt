#[[
author: Kuang Fangjun <csukuangfj at gmail dot com>
date: June 28, 2018
]]

set(NEEDED_LIBS
        ${OpenCV_LIBS}
        ${GLOG_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        ${matio_LIBRARY}
        ${CAFFE_LIBRARY}
        -lboost_system  # for segnet

        ${PROJECT_NAME}_proto
        )

add_subdirectory(core)
add_subdirectory(tools)
add_subdirectory(gantry)
add_subdirectory(normal)

