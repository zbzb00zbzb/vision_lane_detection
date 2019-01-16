/**
 * @file MergeLines.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date August 02, 2018
 */

#include <gtest/gtest.h>
#include <opencv2/core.hpp>

#include <memory>
#include <vector>

#include "test_common.h"

#include "lane_line/Drawing.h"
#include "lane_line/MergeLines.h"
#include "lane_line/Point.h"

class MergeLinesTest : public ::testing::Test
{
 public:
    void SetUp() override
    {
        tt::CurveFittingProto curve_fitting_proto;
        curve_fitting_proto.set_minimum_points(4);
        curve_fitting_proto.set_threshold(3);
        curve_fitting_proto.set_confidence(0.98);
        curve_fitting_proto.set_max_iterations(100);
        curve_fitting_proto.set_name("line");

        tt::MergeLinesProto merge_lines_proto;
        merge_lines_proto.set_d_diff_in_pixel(10);
        merge_lines_proto.set_theta_diff_in_degree(10);

        merge_lines_ = tt::MergeLines::create(merge_lines_proto,
                                              curve_fitting_proto);
    }
 protected:
    std::shared_ptr<tt::MergeLines> merge_lines_;
};

TEST_F(MergeLinesTest, do_merge)
{
    // merge curve1, curve2 and curve3
    // theta1-theta2  is 2
    // theta2-theta3  is 3


    // merge curve4 and curve5
    // theta4-theta5 is 5

    // curve6

    float theta1 = static_cast<float>(45.f/180*CV_PI);
    float theta2 = static_cast<float>(43.f/180*CV_PI);
    float theta3 = static_cast<float>(40.f/180*CV_PI);

    float theta4 = static_cast<float>(65.f/180*CV_PI);
    float theta5 = static_cast<float>(60.f/180*CV_PI);

    float theta6 = static_cast<float>(135.f/180*CV_PI);

    float d1 = 400;
    float d2 = 403;
    float d3 = 405;

    float d4 = 408;
    float d5 = 402;

    float d6 = 302;

    float start_x = 50;

    auto p1 = tt::generatePoints(theta1, d1, start_x);
    auto p2 = tt::generatePoints(theta2, d2, start_x + 50);
    auto p3 = tt::generatePoints(theta3, d3, start_x + 100);

    auto p4 = tt::generatePoints(theta4, d4, start_x + 150);
    auto p5 = tt::generatePoints(theta5, d5, start_x + 200);

    auto p6 = tt::generatePoints(theta6, d6, start_x + 250);

    auto curve1 = merge_lines_->getCurveFitting()->fitCurve(p1);
    auto curve2 = merge_lines_->getCurveFitting()->fitCurve(p2);
    auto curve3 = merge_lines_->getCurveFitting()->fitCurve(p3);
    auto curve4 = merge_lines_->getCurveFitting()->fitCurve(p4);
    auto curve5 = merge_lines_->getCurveFitting()->fitCurve(p5);
    auto curve6 = merge_lines_->getCurveFitting()->fitCurve(p6);

    std::vector<std::shared_ptr<tt::Curve> > curves;
    curves.emplace_back(curve1);
    curves.emplace_back(curve3);
    curves.emplace_back(curve4);
    curves.emplace_back(curve2);
    curves.emplace_back(curve6);
    curves.emplace_back(curve5);

    merge_lines_->doMerge(curves);

    auto merged = merge_lines_->getResult();
    EXPECT_EQ(merged.size(), 3);  // only three curves remain


    cv::Mat image = cv::Mat::zeros(800, 600, CV_8UC3);

    tt::Drawing::lineInPlace(image, *merged[0]->getLinePtr());

    tt::Drawing::listOfPointsInPlace(image, p1, 2, tt::Drawing::green(), 1);
    tt::Drawing::listOfPointsInPlace(image, p2, 2, tt::Drawing::blue(), 1);
    tt::Drawing::listOfPointsInPlace(image, p3, 2, tt::Drawing::white(), 1);

    cv::imshow("image", image);
    cv::waitKey(0);
}

