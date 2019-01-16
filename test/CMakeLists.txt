#[[
author: Kuang Fangjun <csukuangfj at gmail dot com>
date: May 30, 2018
]]

if(NOT ENABLE_GTEST)
    message(STATUS "google test is disabled")
    return()
endif()

include(${PROJECT_SOURCE_DIR}/cmake/gtest.cmake)

#file(GLOB srcs *.cpp)
set(srcs
        test_main.cpp
        test_common.cpp

        CubicCurve.cpp
        Curve.cpp
        CurveFitting.cpp
        DateTime.cpp
        Drawing.cpp
        Evaluation.cpp
        FileSystem.cpp
        IPMParameters.cpp
        IPMTransformation.cpp
        Line.cpp
        MergeLines.cpp
        Narrowing.cpp
        Point.cpp
        PointsExtraction.cpp
        Preprocessing.cpp
        QuadraticCurve.cpp
        RemoveLines.cpp
        Spline.cpp
        Transformation.cpp

        protobuf.cpp
        segnet.cpp
        )

set(testname "test-${PROJECT_NAME}")
add_executable(${testname} ${srcs})
target_link_libraries(${testname} ${PROJECT_NAME}_core ${GTEST_LIBRARIES})

add_executable(raw-image-test raw_image_test.cpp)
target_link_libraries(raw-image-test ${GTEST_LIBRARIES} ${PROJECT_NAME}_core)

add_executable(cnn-caffe-test test_cnn_caffe_main.cpp)
target_link_libraries(cnn-caffe-test ${GTEST_LIBRARIES} ${PROJECT_NAME}_core)

add_executable(compare-ipm-raw-cnn compare_ipm_raw_cnn.cpp)
target_link_libraries(compare-ipm-raw-cnn ${GTEST_LIBRARIES} ${PROJECT_NAME}_core)


add_executable(compute-metrics compute_eval_metrics.cpp)
target_link_libraries(compute-metrics ${PROJECT_NAME}_core)

add_subdirectory(eval)
