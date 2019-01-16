#[[
author: Kuang Fangjun <csukuangfj at gmail dot com>
date: June 28, 2018
]]

add_executable(IPMToolExecutable IpmToolExecutable.cpp)
target_link_libraries(IPMToolExecutable ${PROJECT_NAME}_core)

if(ENABLE_ROS)
    add_executable(image_view image_view.cpp)
    target_link_libraries(image_view ${catkin_LIBRARIES} ${OpenCV_LIBS})

    add_executable(draw_detected_lines draw_detected_lines_node.cpp)
    target_link_libraries(draw_detected_lines ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PROJECT_NAME}_core)

    install(TARGETS draw_detected_lines image_view
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
            )

    install(FILES ${PROJECT_SOURCE_DIR}/launch/image_view.launch
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
            )

    add_executable(image_publisher image_publisher.cpp)
    target_link_libraries(image_publisher ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PROJECT_NAME}_core)
endif()
