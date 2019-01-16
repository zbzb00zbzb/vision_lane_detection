#[[
author: Kuang Fangjun <csukuangfj at gmail dot com>
date: August 01, 2018
]]

set(srcs
        general_main.cpp
        )

add_executable(general_main ${srcs})
target_link_libraries(general_main ${PROJECT_NAME}_core)

if(ENABLE_ROS)
    add_executable(${PROJECT_NAME}_general ros_node.cpp)
    target_link_libraries(${PROJECT_NAME}_general
            ${PROJECT_NAME}_core
            ${catkin_LIBRARIES}
            )
endif()
