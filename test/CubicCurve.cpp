/**
 * @file CubicCurve.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date July 04, 2018
 *
 */

#include <glog/logging.h>
#include <gtest/gtest.h>

#include <vector>

#include "lane_line/common.h"

#include "lane_line/CubicCurve.h"
#include "lane_line/CurveFitting.h"
#include "lane_line/Drawing.h"

class CubicCurveTest: public ::testing::Test
{
 protected:
    void SetUp() override
    {
        curve_ = tt::Curve::create("cubic");
        CHECK_NOTNULL(curve_.get());
    }
    std::shared_ptr<tt::Curve> curve_;
    tt::CubicCurve c_;
};


TEST_F(CubicCurveTest, case_1)
{
    CHECK_EQ(curve_->type(), "cubic");
}

TEST_F(CubicCurveTest, init_with_4_points)
{
    std::vector<tt::Point> points;
    float a = 200;
    float b = 0.25;
    float c = 3;
    float d = -0.3;

    for (int i = 0; i < 4; i++)
    {
        float x = i;
        float y;
        y = a + b*i + c*i*i + d*i*i*i;
        points.emplace_back(x, y);
    }

    c_.initWithPoints(points);

    LOG(INFO) << c_.toString();

    const float eps = 1e-4;
    EXPECT_NEAR(c_.poly().a_, a, eps);
    EXPECT_NEAR(c_.poly().b_, b, eps);
    EXPECT_NEAR(c_.poly().c_, c, eps);
    EXPECT_NEAR(c_.poly().d_, d, eps);
}

TEST_F(CubicCurveTest, init_with_N_points)
{
    std::vector<tt::Point> points;
    float a = 200;
    float b = 0.25;
    float c = 3;
    float d = -0.3;

    for (int i = 0; i < 10; i++)
    {
        float x = i;
        float y;
        y = a + b*i + c*i*i + d*i*i*i;
        y += cv::theRNG().uniform(-5, 5);
        points.emplace_back(x, y);
    }

    c_.initWithPoints(points);

    LOG(INFO) << c_.toString();

    const float eps = 5;        // cubic curve is very sensitive to noise
    EXPECT_NEAR(c_.poly().a_, a, eps);
    EXPECT_NEAR(c_.poly().b_, b, eps);
    EXPECT_NEAR(c_.poly().c_, c, eps);
    EXPECT_NEAR(c_.poly().d_, d, eps);
}

TEST_F(CubicCurveTest, init_with_N_points_ransac)
{
    std::vector<tt::Point> points;
    float a = 200;
    float b = 0.25;
    float c = 3;
    float d = -0.3;

    for (int i = 0; i < 10; i++)
    {
        float x = i;
        float y;
        y = a + b*i + c*i*i + d*i*i*i;
        y += cv::theRNG().uniform(-4., 4.);
        points.emplace_back(x, y);
    }

    c_.initWithPoints(points);
    LOG(INFO) << "\nleast square:\n" << c_.toString();

    tt::CurveFittingProto proto;

    proto.set_threshold(3);
    proto.set_max_iterations(1000);
    proto.set_confidence(0.95f);
    proto.set_name("cubic");
    proto.set_minimum_points(5);

    auto fit = tt::CurveFitting::create(proto);
    curve_ = fit->fitCurve(points);

    LOG(INFO) << "\nransac:\n" << curve_->toString();
}

TEST_F(CubicCurveTest, init_with_N_points_line)
{
    std::vector<tt::Point> points;
    float a = 200;
    float b = 0.25;
    float c = 0;
    float d = 0;

    for (int i = 0; i < 20; i++)
    {
        float x = i;
        float y;
        y = a + b*i + c*i*i + d*i*i*i;
        y += cv::theRNG().uniform(-1, 1);
        points.emplace_back(x, y);
    }

    c_.initWithPoints(points);

    LOG(INFO) << c_.toString();

    const float eps = 2;        // cubic curve is very sensitive to noise
    EXPECT_NEAR(c_.poly().a_, a, eps);
    EXPECT_NEAR(c_.poly().b_, b, eps);
    EXPECT_NEAR(c_.poly().c_, c, eps);
    EXPECT_NEAR(c_.poly().d_, d, eps);
}
