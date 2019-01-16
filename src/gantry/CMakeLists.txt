#[[
author: Kuang Fangjun <csukuangfj at gmail dot com>
date: June 28, 2018
]]

set(srcs
        gantry_main.cpp
        )

add_executable(gantry_main ${srcs})
target_link_libraries(gantry_main ${PROJECT_NAME}_core)

if(ENABLE_ROS)
    add_executable(${PROJECT_NAME}_gantry ros_node.cpp)
    target_link_libraries(${PROJECT_NAME}_gantry
            ${PROJECT_NAME}_core
            ${catkin_LIBRARIES}
            )
    install(TARGETS ${PROJECT_NAME}_gantry
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
            )
    install(FILES ${PROJECT_SOURCE_DIR}/launch/gantry.launch
             DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
             )
    install(#FILES
            #${PROJECT_SOURCE_DIR}/config/proto.txt
            #${PROJECT_SOURCE_DIR}/config/proto.txt-anqiao
            #${PROJECT_SOURCE_DIR}/config/gantry.yaml
            #${PROJECT_SOURCE_DIR}/config/custom_rosconsole.conf
            #DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
            DIRECTORY
            ${PROJECT_SOURCE_DIR}/config/
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config/
            )
    
    install(DIRECTORY
            ${PROJECT_SOURCE_DIR}/tools/
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/tools/
            )

    install(DIRECTORY
            ${PROJECT_SOURCE_DIR}/models/
            #${PROJECT_SOURCE_DIR}/models/after_bn_calc2.caffemodel
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models/
            )
    install(FILES
            ${PROJECT_SOURCE_DIR}/lib/libcaffe.so
            ${PROJECT_SOURCE_DIR}/lib/libcaffe.so.1.0.0-rc3
            DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            )
endif()
