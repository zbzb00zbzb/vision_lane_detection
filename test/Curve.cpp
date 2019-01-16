/**
 * @file Curve.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 11, 2018
 */


#include <gtest/gtest.h>

#include "lane_line/Curve.h"
#include "lane_line/Line.h"

class CurveTest : public ::testing::Test
{
 public:
    void SetUp() override
    {}
};

TEST_F(CurveTest, line_is_similar_to)
{
    auto line1 = std::make_shared<tt::Line>();
    auto line2 = std::make_shared<tt::Line>();

    float thresholds[2];

    line1->initWithDegree(30, 15);  // degree: 30, distance: 15
    line2->initWithDegree(33, 15);  // degree: 30, distance: 15

    bool res;
    thresholds[0] = 4;
    thresholds[1] = 3;
    res = line1->isSimilarTo(line2, thresholds, 2);  // delta_theta: 3 < 4
    EXPECT_TRUE(res);

    thresholds[0] = 2;
    thresholds[1] = 3;
    res = line1->isSimilarTo(line2, thresholds, 2);  // delta_theta: 3 > 2
    EXPECT_FALSE(res);

    //------------------------------------------------------------

    line1->initWithDegree(30, 20);  // degree: 30, distance: 15
    line2->initWithDegree(33, 15);  // degree: 30, distance: 15

    thresholds[0] = 4;
    thresholds[1] = 3;
    // delta_theta: 3 < 4, delta_d = 5 > 3
    res = line1->isSimilarTo(line2, thresholds, 2);
    EXPECT_FALSE(res);

    thresholds[0] = 4;
    thresholds[1] = 6;
    // delta_theta: 3 < 4, delta_d = 5 < 6
    res = line1->isSimilarTo(line2, thresholds, 2);
    EXPECT_TRUE(res);

    thresholds[0] = 1;
    thresholds[1] = 6;
    // delta_theta: 3 > 1, delta_d = 5 < 6
    res = line1->isSimilarTo(line2, thresholds, 2);
    EXPECT_FALSE(res);
}

TEST_F(CurveTest, get_confidence)
{
    const float eps = 1e-3;
    auto curve = std::make_shared<tt::Line>();
    float mean = 100;
    float std_dev = 30;

    curve->inliers().resize(100);
    float res = curve->getConfidence(mean, std_dev);
    float expected = 1;

    EXPECT_NEAR(res, expected, eps);

    curve->missed_num()++;
    res = curve->getConfidence(mean, std_dev);
    expected = 0.5;

    EXPECT_NEAR(res, expected, eps);

    curve->missed_num() = 0;

    curve->inliers().resize(90);
    res = curve->getConfidence(mean, std_dev);
    expected = 0.9459594f;
    EXPECT_NEAR(res, expected, eps);

    curve->inliers().resize(80);
    res = curve->getConfidence(mean, std_dev);
    expected = 0.800737f;
    EXPECT_NEAR(res, expected, eps);

    curve->inliers().resize(70);
    res = curve->getConfidence(mean, std_dev);
    expected = 0.6065306f;
    EXPECT_NEAR(res, expected, eps);

    curve->inliers().resize(60);
    res = curve->getConfidence(mean, std_dev);
    expected = 0.41111229f;
    EXPECT_NEAR(res, expected, eps);

    curve->inliers().resize(50);
    res = curve->getConfidence(mean, std_dev);
    expected = 0.249352208f;
    EXPECT_NEAR(res, expected, eps);
}
