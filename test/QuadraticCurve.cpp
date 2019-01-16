/**
 * @file QuadraticCurve.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date July 04, 2018
 *
 */

#include <glog/logging.h>
#include <gtest/gtest.h>

#include <vector>

#include "lane_line/common.h"

#include "lane_line/CurveFitting.h"
#include "lane_line/Drawing.h"
#include "lane_line/QuadraticCurve.h"

class QuadraticCurveTest : public ::testing::Test
{
 protected:
    void SetUp() override
    {
        curve_ = tt::Curve::create("quadratic");
        CHECK_NOTNULL(curve_.get());
    }
    std::shared_ptr<tt::Curve> curve_;
    tt::QuadraticCurve q_;
};

TEST_F(QuadraticCurveTest, case_1)
{
    CHECK_EQ(curve_->type(), "quadratic");
}

TEST_F(QuadraticCurveTest, init_with_points)
{
    std::vector<tt::Point> points;
    float a = 2000;
    float b = -15;
    float c = 0.03;

    for (int i = 0; i < 3; i++)
    {
        points.emplace_back(i, a + b*i + c*i*i);
    }

    q_.initWithPoints(points);
    const float eps = 1e-3;
    EXPECT_NEAR(q_.poly().a_, a, eps);
    EXPECT_NEAR(q_.poly().b_, b, eps);
    EXPECT_NEAR(q_.poly().c_, c, eps);

    LOG(INFO) << q_.toString();

    points.clear();
    for (int i = 0; i < 500; i += 20)
    {
        points.emplace_back(i, q_.compute(i));
    }

    cv::Mat m = cv::Mat::zeros(1000, 1000, CV_8UC3);
    tt::Drawing::polyLineInPlace(m, points, tt::Drawing::green(), 2);

    for (const auto &p : points)
    {
        tt::Drawing::circleInPlace(m, p, 3, tt::Drawing::red(), -1);
    }

    cv::namedWindow("quadratic curve 3", cv::WINDOW_NORMAL);
    cv::imshow("quadratic curve 3", m);
    cv::waitKey(1);
}


TEST_F(QuadraticCurveTest, init_with_N_points)
{
    std::vector<tt::Point> points;
    float a = 2000;
    float b = -15;
    float c = 0.03;

    for (int i = 0; i < 500; i += 5)
    {
        float x = i;
        float y;
        y = a + b*x + c*x*x;
        y += cv::theRNG().uniform(-50, 50);

        points.emplace_back(x, y);
    }

    q_.initWithPoints(points);

    LOG(INFO) << q_.toString();

    cv::Mat m = cv::Mat::zeros(1000, 1000, CV_8UC3);

    for (const auto &p : points)
    {
        tt::Drawing::circleInPlace(m, p, 3, tt::Drawing::white(), -1);
    }

    points.clear();
    for (int i = 0; i < 500; i += 20)
    {
        points.emplace_back(i, q_.compute(i));
    }
    tt::Drawing::polyLineInPlace(m, points, tt::Drawing::green(), 2);


    cv::namedWindow("quadratic curve N", cv::WINDOW_NORMAL);
    cv::imshow("quadratic curve N", m);
    cv::waitKey(1);
}

TEST_F(QuadraticCurveTest, curve_fitting)
{
    std::vector<tt::Point> points;
    float a = 2000;
    float b = -15;
    float c = 0.03;

    for (int i = 0; i < 500; i += 5)
    {
        float x = i;
        float y;
        y = a + b*x + c*x*x;
        y += cv::theRNG().uniform(-50, 50);

        points.emplace_back(x, y);
    }

    q_.initWithPoints(points);

    std::vector<tt::Point> points2 = points;

    LOG(INFO) << "\nleast square: " << q_.toString();

    tt::CurveFittingProto proto;

    proto.set_threshold(15);
    proto.set_max_iterations(1000);
    proto.set_confidence(0.95f);
    proto.set_name("quadratic");
    proto.set_minimum_points(5);

    auto fit = tt::CurveFitting::create(proto);
    curve_ = fit->fitCurve(points);

    LOG(INFO) << "\nRANSAC: " << curve_->toString();

    cv::Mat least = cv::Mat::zeros(1000, 1000, CV_8UC3);
    points.clear();
    for (int i = 0; i < 500; i += 10)
    {
        points.emplace_back(i, q_.compute(i));
    }
    tt::Drawing::polyLineInPlace(least, points, tt::Drawing::green(), 2);

    // for (const auto &p : q_.inliers())
    for (const auto &p : points2)
    {
        tt::Drawing::circleInPlace(least, p, 3, tt::Drawing::red(), -1);
    }

    cv::Mat ransac = cv::Mat::zeros(1000, 1000, CV_8UC3);

    auto *cc = dynamic_cast<tt::QuadraticCurve*>(curve_.get());
    points.clear();
    for (int i = 0; i < 500; i += 20)
    {
        points.emplace_back(i, cc->compute(i));
    }
    tt::Drawing::polyLineInPlace(ransac, points, tt::Drawing::green(), 2);
    // for (const auto &p : curve_->inliers())
    for (const auto &p : points2)
    {
        tt::Drawing::circleInPlace(ransac, p, 3, tt::Drawing::red(), -1);
    }

    cv::namedWindow("least square", cv::WINDOW_NORMAL);
    cv::imshow("least square", least);

    cv::namedWindow("ransac", cv::WINDOW_NORMAL);
    cv::imshow("ransac", ransac);

    cv::waitKey(1);
}

TEST_F(QuadraticCurveTest, fit_line)
{
    std::vector<tt::Point> points;
    float a = 800;
    float b = -1;
    float c = 0;

    for (int i = 0; i < 500; i += 5)
    {
        float x = i;
        float y;
        y = a + b*x + c*x*x;
        y += cv::theRNG().uniform(-50, 50);

        points.emplace_back(x, y);
    }

    q_.initWithPoints(points);

    std::vector<tt::Point> points2 = points;

    LOG(INFO) << "\nleast square (line): " << q_.toString();

    points.clear();
    for (int i = 0; i < 1000; i += 20)
    {
        points.emplace_back(i, q_.compute(i));
    }

    cv::Mat m = cv::Mat::zeros(1000, 1000, CV_8UC3);
    tt::Drawing::polyLineInPlace(m, points, tt::Drawing::green(), 2);

    for (const auto &p : points2)
    {
        tt::Drawing::circleInPlace(m, p, 3, tt::Drawing::red(), -1);
    }

    cv::namedWindow("quadratic curve line", cv::WINDOW_NORMAL);
    cv::imshow("quadratic curve line", m);
    cv::waitKey(0);
}

TEST_F(QuadraticCurveTest, point_distance)
{
    std::vector<tt::Point> points;
    float a = 2000;
    float b = -15;
    float c = 0.03;

    for (int i = 0; i < 3; i++)
    {
        points.emplace_back(i, a + b*i + c*i*i);
    }

    q_.initWithPoints(points);

    float offset = 5;
    float x = 30;
    float y = q_.compute(x) + offset;

    float actual_dist;
    actual_dist = q_.pointAbsDistanceToThisCurve({x, y});
    EXPECT_NEAR(actual_dist, offset, 1e-5);
}
