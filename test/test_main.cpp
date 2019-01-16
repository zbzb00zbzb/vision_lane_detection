#include <glog/logging.h>
#include <gtest/gtest.h>

int main(int argc, char **argv)
{
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;
    ::testing::InitGoogleTest(&argc, argv);
    google::InitGoogleLogging(argv[0]);
#if 1
//    testing::GTEST_FLAG(filter) = "CubicCurveTest.*";
//    testing::GTEST_FLAG(filter) = "CurveTest.*";
//    testing::GTEST_FLAG(filter) = "CurveFittingTest.*";
//    testing::GTEST_FLAG(filter) = "CurveFittingTest.fit_line";
//    testing::GTEST_FLAG(filter) = "DateTimeTest.*";
//    testing::GTEST_FLAG(filter) = "DrawingTest.*";
      testing::GTEST_FLAG(filter) = "EvaluationTest.*";
//    testing::GTEST_FLAG(filter) = "FileSystemTest.*";
//    testing::GTEST_FLAG(filter) = "IPMParametersTest.*";
//    testing::GTEST_FLAG(filter) = "IPMTransformationTest.*";
//    testing::GTEST_FLAG(filter) = "IPMTransformationTest.precomputed_H";
//    testing::GTEST_FLAG(filter) = "LineTest.*";
//    testing::GTEST_FLAG(filter) = "MergeLinesTest.*";
//    testing::GTEST_FLAG(filter) = "NarrowingTest.*";
//    testing::GTEST_FLAG(filter) = "PointTest.*";
//    testing::GTEST_FLAG(filter) = "PointsExtractionTest.*";
//    testing::GTEST_FLAG(filter) = "PreprocessingTest.*";
//    testing::GTEST_FLAG(filter) = "ProtobufTest.*";
//    testing::GTEST_FLAG(filter) = "RemoveLinesTest.*";
//    testing::GTEST_FLAG(filter) = "QuadraticCurveTest.*";
//    testing::GTEST_FLAG(filter) = "SegnetTest.*";
//    testing::GTEST_FLAG(filter) = "SplineTest.*";
//    testing::GTEST_FLAG(filter) = "TransformationTest.*";
#endif
    return RUN_ALL_TESTS();
}
