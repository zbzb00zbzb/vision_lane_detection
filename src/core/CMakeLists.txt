#[[
author: Kuang Fangjun <csukuangfj at gmail dot com>
date: June 28, 2018
]]

message(STATUS "start cmake core ")
set(srcs
        protobuf_util.cpp
        tools.cpp

        CubicCurve.cpp
        Curve.cpp
        CurveFitting.cpp
        DateTime.cpp
        DistTransNarrowing.cpp
        Drawing.cpp
        Evaluation.cpp
        FileSystem.cpp
        FilterLines.cpp
        GantryLaneDetector.cpp
        IPMParameters.cpp
        IPMTransformation.cpp
        KalmanLineFilter.cpp
        LaneLineDetector.cpp
        Line.cpp
        MergeLines.cpp
        Narrowing.cpp
        Point.cpp
        PointsExtraction.cpp
        Preprocessing.cpp
        QuadraticCurve.cpp
        RemoveLines.cpp
        RowScanNarrowing.cpp
        SegnetPreprocessing.cpp
        Spline.cpp
        Transformation.cpp
        visualization.cpp

        ${PROJECT_SOURCE_DIR}/3rdparty/cnn/CnnCaffe.cpp
        ${PROJECT_SOURCE_DIR}/3rdparty/cnn/CnnInterface.cpp
        )

set(lib ${PROJECT_NAME}_core)
add_library(${lib} ${MY_TYPE} ${srcs})
target_link_libraries(${lib} ${NEEDED_LIBS})
