/**
 * @file Line.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date May 30, 2018
 */

#include <gtest/gtest.h>
#include <memory>

#include "lane_line/Drawing.h"
#include "lane_line/Line.h"

using tt::Line;
using tt::Point;
using tt::Drawing;

class LineTest : public ::testing::Test
{
 public:
    void SetUp() override
    {
        line_ = std::make_shared<Line>();
        image_ = cv::Mat::zeros(cv::Size(800, 600), CV_8UC3);
    }

 protected:
    std::shared_ptr<Line> line_;
    Point p1_;
    Point p2_;

    cv::Mat image_;  // for visualization purpose

    const float eps_ = 1e-4;
};

TEST_F(LineTest, init)
{
    cv::destroyAllWindows();

    line_.reset(new Line());
    EXPECT_NEAR(line_->getThetaInRadian(), 0, eps_);
    EXPECT_NEAR(line_->getD(), 0, eps_);

    line_.reset(new Line(1, 2));
    EXPECT_NEAR(line_->getThetaInRadian(), 1, eps_);
    EXPECT_NEAR(line_->getD(), 2, eps_);

    EXPECT_DEATH(line_->initWithRadian(CV_2PI, 1), "Invalid theta");
    EXPECT_DEATH(line_->initWithRadian(CV_2PI+1, 1), "Invalid theta");
    EXPECT_DEATH(line_->initWithRadian(-1e-5, 1), "Invalid theta");

    EXPECT_DEATH(line_->initWithRadian(0, -1), "Invalid d");  // negative d

    EXPECT_DEATH(line_->initWithDegree(360, 1), "Invalid theta");
    EXPECT_DEATH(line_->initWithDegree(361, 1), "Invalid theta");
    EXPECT_DEATH(line_->initWithDegree(-1e-5, 1), "Invalid theta");

    EXPECT_DEATH(line_->initWithDegree(0, -1), "Invalid d");  // negative d
}

TEST_F(LineTest, horizontal_line_0_degree_positive_y)
{
    float expected_theta;
    float expected_d;

    float actual_theta;
    float actual_d;

    p1_ = {300, 300};
    p2_ = {100, 300};

    line_->initWithPoints(p1_, p2_);

    expected_theta = 90;
    actual_theta = line_->getThetaInDegree();
    EXPECT_NEAR(expected_theta, actual_theta, eps_);

    expected_d = 300;
    actual_d = line_->getD();
    EXPECT_NEAR(expected_d, actual_d, eps_);

    // flip the two points, the result should be the same
    line_->initWithPoints(p2_, p1_);

    expected_theta = 90;
    actual_theta = line_->getThetaInDegree();
    EXPECT_NEAR(expected_theta, actual_theta, eps_);

    expected_d = 300;
    actual_d = line_->getD();
    EXPECT_NEAR(expected_d, actual_d, eps_);

    auto result = Drawing::line(image_, p1_, p2_, Drawing::red(), 1);
    cv::imshow("line-0-degree-positive-y", result);
    cv::waitKey(1);
}

TEST_F(LineTest, horizontal_line_0_degree_negative_y)
{
    float expected_theta;
    float expected_d;

    float actual_theta;
    float actual_d;

    p1_ = {300, -300};
    p2_ = {100, -300};

    line_->initWithPoints(p1_, p2_);

    expected_theta = 270;
    actual_theta = line_->getThetaInDegree();
    EXPECT_NEAR(expected_theta, actual_theta, eps_);

    expected_d = 300;
    actual_d = line_->getD();
    EXPECT_NEAR(expected_d, actual_d, eps_);

    // flip the two points, the result should be the same
    line_->initWithPoints(p2_, p1_);

    expected_theta = 270;
    actual_theta = line_->getThetaInDegree();
    EXPECT_NEAR(expected_theta, actual_theta, eps_);

    expected_d = 300;
    actual_d = line_->getD();
    EXPECT_NEAR(expected_d, actual_d, eps_);

    auto result = Drawing::line(image_, p1_, p2_, Drawing::red(), 1);
    cv::imshow("line-0-degree-negative-y", result);  // should display nothing
    cv::waitKey(1);
}

TEST_F(LineTest, vertical_line_90_degree_positive_x)
{
    float expected_theta;
    float expected_d;

    float actual_theta;
    float actual_d;

    p1_ = {300, 100};
    p2_ = {300, 300};

    line_->initWithPoints(p1_, p2_);

    expected_theta = 0;
    actual_theta = line_->getThetaInDegree();
    EXPECT_NEAR(expected_theta, actual_theta, eps_);

    expected_d = 300;
    actual_d = line_->getD();
    EXPECT_NEAR(expected_d, actual_d, eps_);

    // flip the two points, the result should be the same
    line_->initWithPoints(p2_, p1_);

    expected_theta = 0;
    actual_theta = line_->getThetaInDegree();
    EXPECT_NEAR(expected_theta, actual_theta, eps_);

    expected_d = 300;
    actual_d = line_->getD();
    EXPECT_NEAR(expected_d, actual_d, eps_);

    auto result = Drawing::line(image_, p1_, p2_, Drawing::red(), 1);
    cv::imshow("line-90-degree-positive-x", result);
    cv::waitKey(1);
}

TEST_F(LineTest, vertical_line_90_degree_negative_x)
{
    float expected_theta;
    float expected_d;

    float actual_theta;
    float actual_d;

    p1_ = {-300, 100};
    p2_ = {-300, 300};

    line_->initWithPoints(p1_, p2_);

    expected_theta = 180;
    actual_theta = line_->getThetaInDegree();
    EXPECT_NEAR(expected_theta, actual_theta, eps_);

    expected_d = 300;
    actual_d = line_->getD();
    EXPECT_NEAR(expected_d, actual_d, eps_);

    // flip the two points, the result should be the same
    line_->initWithPoints(p2_, p1_);

    expected_theta = 180;
    actual_theta = line_->getThetaInDegree();
    EXPECT_NEAR(expected_theta, actual_theta, eps_);

    expected_d = 300;
    actual_d = line_->getD();
    EXPECT_NEAR(expected_d, actual_d, eps_);

    auto result = Drawing::line(image_, p1_, p2_, Drawing::red(), 1);
    cv::imshow("line-90-degree-negative-x", result);
    cv::waitKey(1);
}

TEST_F(LineTest, line_45_degree_134)
{
    float expected_theta;
    float expected_d;

    float actual_theta;
    float actual_d;

    p1_ = {0, -100};
    p2_ = {500, 400};

    line_->initWithPoints(p1_, p2_);

    expected_theta = 315;
    actual_theta = line_->getThetaInDegree();
    EXPECT_NEAR(expected_theta, actual_theta, eps_);

    expected_d = 100.0f / sqrtf(2);
    actual_d = line_->getD();
    EXPECT_NEAR(expected_d, actual_d, eps_);

    // flip the two points, the result should be the same
    line_->initWithPoints(p2_, p1_);

    expected_theta = 315;
    actual_theta = line_->getThetaInDegree();
    EXPECT_NEAR(expected_theta, actual_theta, eps_);

    expected_d = 100.0f / sqrtf(2);
    actual_d = line_->getD();
    EXPECT_NEAR(expected_d, actual_d, eps_);

    auto result = Drawing::line(image_, p1_, p2_, Drawing::red(), 1);
    Drawing::lineInPlace(result, cv::Point2f(0, 0),
                         cv::Point2f(result.cols, result.cols),
                         Drawing::white(), 1);
    cv::imshow("line-45-degree-134", result);
    cv::waitKey(1);
}

TEST_F(LineTest, line_45_degree_123)
{
    float expected_theta;
    float expected_d;

    float actual_theta;
    float actual_d;

    p1_ = {0, 100};
    p2_ = {500, 600};

    line_->initWithPoints(p1_, p2_);

    expected_theta = 135;
    actual_theta = line_->getThetaInDegree();
    EXPECT_NEAR(expected_theta, actual_theta, eps_);

    expected_d = 100.0f / sqrtf(2);
    actual_d = line_->getD();
    EXPECT_NEAR(expected_d, actual_d, eps_);

    // flip the two points, the result should be the same
    line_->initWithPoints(p2_, p1_);

    expected_theta = 135;
    actual_theta = line_->getThetaInDegree();
    EXPECT_NEAR(expected_theta, actual_theta, eps_);

    expected_d = 100.0f / sqrtf(2);
    actual_d = line_->getD();
    EXPECT_NEAR(expected_d, actual_d, eps_);

    auto result = Drawing::line(image_, p1_, p2_, Drawing::red(), 1);
    Drawing::lineInPlace(result, cv::Point2f(0, 0),
                         cv::Point2f(result.cols, result.cols),
                         Drawing::white(), 1);
    cv::imshow("line-45-degree-123", result);
    cv::waitKey(1);
}

TEST_F(LineTest, line_135_degree_124)
{
    float expected_theta;
    float expected_d;

    float actual_theta;
    float actual_d;

    p1_ = {0, 400};
    p2_ = {400, 0};

    line_->initWithPoints(p1_, p2_);

    expected_theta = 45;
    actual_theta = line_->getThetaInDegree();
    EXPECT_NEAR(expected_theta, actual_theta, eps_);

    expected_d = 400.0f / sqrtf(2);
    actual_d = line_->getD();
    EXPECT_NEAR(expected_d, actual_d, eps_);

    // flip the two points, the result should be the same
    line_->initWithPoints(p2_, p1_);

    expected_theta = 45;
    actual_theta = line_->getThetaInDegree();
    EXPECT_NEAR(expected_theta, actual_theta, eps_);

    expected_d = 400.0f / sqrtf(2);
    actual_d = line_->getD();
    EXPECT_NEAR(expected_d, actual_d, eps_);

    auto result = Drawing::line(image_, p1_, p2_, Drawing::red(), 1);
    Drawing::lineInPlace(result, cv::Point2f(0, 0),
                         cv::Point2f(result.cols, result.cols),
                         Drawing::white(), 1);
    cv::imshow("line-135-degree-124", result);
    cv::waitKey(1);
}

TEST_F(LineTest, line_135_degree_234)
{
    float expected_theta;
    float expected_d;

    float actual_theta;
    float actual_d;

    p1_ = {0, -100};
    p2_ = {-100, 0};

    line_->initWithPoints(p1_, p2_);

    expected_theta = 225;
    actual_theta = line_->getThetaInDegree();
    EXPECT_NEAR(expected_theta, actual_theta, eps_);

    expected_d = 100.0f / sqrtf(2);
    actual_d = line_->getD();
    EXPECT_NEAR(expected_d, actual_d, eps_);

    // flip the two points, the result should be the same
    line_->initWithPoints(p2_, p1_);

    expected_theta = 225;
    actual_theta = line_->getThetaInDegree();
    EXPECT_NEAR(expected_theta, actual_theta, eps_);

    expected_d = 100.0f / sqrtf(2);
    actual_d = line_->getD();
    EXPECT_NEAR(expected_d, actual_d, eps_);

    auto result = Drawing::line(image_, p1_, p2_, Drawing::red(), 1);
    cv::imshow("line-135-degree-234", result);
    cv::waitKey(1);
}

TEST_F(LineTest, closest_point_on_line)
{
    p1_ = {200, 0};
    p2_ = {0, 200};
    line_->initWithPoints(p1_, p2_);

    Point p(0, 0);
    auto closest_point = line_->pointProjectedOnLine(p);

    Point expected_point(100, 100);
    double diff = cv::norm(cv::Point2f(closest_point) -
                                   cv::Point2f(expected_point));
    EXPECT_NEAR(diff, 0, eps_);

    Line line2;
    line2.initWithPoints(p, closest_point);

    auto image = Drawing::line(image_, p1_, p2_, Drawing::red(), 1);
    Drawing::lineInPlace(image, p, closest_point, Drawing::green(), 1);
    Drawing::circleInPlace(image, closest_point, 1, Drawing::white(), -1);

    cv::imshow("closest point", image);
    cv::waitKey(0);
}

TEST_F(LineTest, point_to_line_distance)
{
    p1_ = {200, 0};
    p2_ = {0, 500};
    line_->initWithPoints(p1_, p2_);

    Point p(0, 0);
    auto closest_point = line_->pointProjectedOnLine(p);
    double expected_distance = cv::norm(cv::Point2f(p) -
                                               cv::Point2f(closest_point));

    double actual_distance = line_->pointAbsDistanceToThisCurve(p);

    EXPECT_NEAR(expected_distance, actual_distance, eps_);
}
