/**
 * @file Drawing.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date May 30, 2018
 */

#include <gtest/gtest.h>

#include <vector>

#include "lane_line/Drawing.h"
#include "lane_line/Spline.h"

using tt::Drawing;

class DrawingTest : public ::testing::Test
{
 public:
    void SetUp() override
    {
        image_ = cv::Mat::zeros(cv::Size(800, 600), CV_8UC3);
    }

 protected:
    cv::Mat image_;
};

TEST_F(DrawingTest, line)
{
    cv::destroyAllWindows();
    cv::Point2f p1(30, 20);
    cv::Point2f p2(500, 300);

    auto result = Drawing::line(image_, p1, p2, Drawing::green(), 1);

    cv::imshow("line", result);
    cv::waitKey(10);
}

TEST_F(DrawingTest, line_in_place)
{
    cv::Point2f p1(30, 20);
    cv::Point2f p2(500, 300);

    auto result = image_.clone();

    Drawing::lineInPlace(result, p1, p2, Drawing::green(), 1);

    p1 = {100, 200};
    p2 = {300, 200};
    Drawing::lineInPlace(result, p1, p2, Drawing::red(), 1);

    p1 = {500, 20};
    p2 = {30, 500};
    Drawing::lineInPlace(result, p1, p2, Drawing::blue(), 1);

    p1 = {800, 0};
    p2 = {0, 600};
    Drawing::lineInPlace(result, p1, p2, Drawing::white(), 1);

    cv::imshow("line_in_place", result);
    cv::waitKey(1);
}

/*
 * y = 200 sin(pi/100 x);
 */
TEST_F(DrawingTest, sine_wave)
{
    tt::Spline spline;
    float A = 200;
    float f = M_PI*1.f/100;
    float offset = 300;
    std::vector<tt::Point> points;
    for (int i = 0; i < 20; i++)
    {
        points.emplace_back(i*50, A*std::sin(f*50*i) + offset);
    }

    auto pp = spline.interpolate(points, 50);
    cv::Mat image = cv::Mat::zeros(600, 600, CV_8UC3);

    Drawing::polyLineInPlace(image, pp, Drawing::green(), 1);

    for (const auto &p : points)
    {
        Drawing::circleInPlace(image, p, 2, Drawing::red(), -1);
    }

    cv::imshow("sine", image);
    cv::waitKey(1);
}

TEST_F(DrawingTest, line_spline)
{
    tt::Spline spline;
    std::vector<tt::Point> points;
    for (int i = 0; i < 500; i += 20)
    {
        points.emplace_back(i, i + cv::theRNG().uniform(-5, 5));
    }

    auto pp = spline.interpolate(points, 50);
    cv::Mat image = cv::Mat::zeros(600, 600, CV_8UC3);

    Drawing::polyLineInPlace(image, pp, Drawing::green(), 1);

    for (const auto &p : points)
    {
        Drawing::circleInPlace(image, p, 2, Drawing::red(), -1);
    }

    cv::imshow("line spline", image);
    cv::waitKey(0);
}
