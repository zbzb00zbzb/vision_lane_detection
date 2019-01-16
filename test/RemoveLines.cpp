/**
 * @file RemoveLines.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date August 02, 2018
 */

#include <gtest/gtest.h>

#include <vector>

#include "lane_line/Drawing.h"
#include "lane_line/RemoveLines.h"

#include "test_common.h"

class RemoveLinesTest : public ::testing::Test
{
 public:
    void SetUp() override
    {
        tt::RemoveLinesProto proto;
        proto.set_minimum_theta_diff_in_degree(8);
        proto.set_minimum_d_diff_in_pixel(40);

        remove_lines_ = tt::RemoveLines::create(proto);
    }

 protected:
    std::shared_ptr<tt::RemoveLines> remove_lines_;
};

TEST_F(RemoveLinesTest, do_remove_two_lines_large_theta)
{
    // remove lines if theta diff is too large
    cv::Mat image = cv::Mat::zeros(800, 600, CV_8UC3);

    float theta1 = static_cast<float>(10.f/180*CV_PI);
    float theta2 = static_cast<float>(19.f/180*CV_PI);

    float d1 = 200;
    float d2 = 500;

    int start_x = 200;
    auto p1 = tt::generatePoints(theta1, d1, start_x);
    auto p11 = tt::generatePoints(theta1, d1, start_x+300);
    p1.insert(p1.end(), p11.begin(), p11.end());

    auto p2 = tt::generatePoints(theta2, d2, start_x + 100);

    auto line1 = std::make_shared<tt::Line>();
    line1->initWithPoints(p1);

    auto line2 = std::make_shared<tt::Line>();
    line2->initWithPoints(p2);

    std::vector<std::shared_ptr<tt::Curve> > curves;
    curves.emplace_back(line2);
    curves.emplace_back(line1);

    remove_lines_->doRemove(curves);
    EXPECT_EQ(remove_lines_->getResult().size(), 1);

    // line1 should be kept.
    EXPECT_NEAR(remove_lines_->getResult()[0]->getLinePtr()->getD(),
                200, 1e-3*200);
}

TEST_F(RemoveLinesTest, do_remove_two_lines_small_d)
{
    // remove lines if theta diff is too large
    cv::Mat image = cv::Mat::zeros(800, 600, CV_8UC3);

    float theta1 = static_cast<float>(10.f/180*CV_PI);
    float theta2 = static_cast<float>(17.f/180*CV_PI);

    float d1 = 200;
    float d2 = 220;

    int start_x = 200;
    auto p1 = tt::generatePoints(theta1, d1, start_x);
    auto p11 = tt::generatePoints(theta1, d1, start_x+300);
    p1.insert(p1.end(), p11.begin(), p11.end());

    auto p2 = tt::generatePoints(theta2, d2, start_x + 100);

    auto line1 = std::make_shared<tt::Line>();
    line1->initWithPoints(p1);

    auto line2 = std::make_shared<tt::Line>();
    line2->initWithPoints(p2);

    std::vector<std::shared_ptr<tt::Curve> > curves;
    curves.emplace_back(line2);
    curves.emplace_back(line1);

    remove_lines_->doRemove(curves);
    EXPECT_EQ(remove_lines_->getResult().size(), 1);

    // line1 should be kept.
    EXPECT_NEAR(remove_lines_->getResult()[0]->getLinePtr()->getD(),
                200, 1e-3*200);
}

TEST_F(RemoveLinesTest, do_remove_no_lines)
{
    // no line is removed
    cv::Mat image = cv::Mat::zeros(800, 600, CV_8UC3);

    float theta1 = static_cast<float>(10.f/180*CV_PI);
    float theta2 = static_cast<float>(17.f/180*CV_PI);

    float d1 = 200;
    float d2 = 241;

    int start_x = 200;
    auto p1 = tt::generatePoints(theta1, d1, start_x);
    auto p11 = tt::generatePoints(theta1, d1, start_x+300);
    p1.insert(p1.end(), p11.begin(), p11.end());

    auto p2 = tt::generatePoints(theta2, d2, start_x + 100);

    auto line1 = std::make_shared<tt::Line>();
    line1->initWithPoints(p1);

    auto line2 = std::make_shared<tt::Line>();
    line2->initWithPoints(p2);

    std::vector<std::shared_ptr<tt::Curve> > curves;
    curves.emplace_back(line2);
    curves.emplace_back(line1);

    remove_lines_->doRemove(curves);
    EXPECT_EQ(remove_lines_->getResult().size(), 2);

    EXPECT_NEAR(remove_lines_->getResult()[0]->getLinePtr()->getD(),
                241, 1e-3*241);
    EXPECT_NEAR(remove_lines_->getResult()[1]->getLinePtr()->getD(),
                200, 1e-3*220);
}

TEST_F(RemoveLinesTest, do_remove_two_from_three)
{
    // line1 and line2: remove line2 due to large theta diff
    // line1 and line3: remove line3 due to small d diff
    cv::Mat image = cv::Mat::zeros(800, 600, CV_8UC3);

    float theta1 = static_cast<float>(10.f/180*CV_PI);
    float theta2 = static_cast<float>(19.f/180*CV_PI);
    float theta3 = static_cast<float>(29.f/180*CV_PI);

    float d1 = 200;
    float d2 = 300;
    float d3 = 220;

    int start_x = 200;
    auto p1 = tt::generatePoints(theta1, d1, start_x);
    auto p11 = tt::generatePoints(theta1, d1, start_x+300);
    p1.insert(p1.end(), p11.begin(), p11.end());

    auto p2 = tt::generatePoints(theta2, d2, start_x + 100);
    auto p3 = tt::generatePoints(theta3, d3, start_x + 30);

    auto line1 = std::make_shared<tt::Line>();
    line1->initWithPoints(p1);

    auto line2 = std::make_shared<tt::Line>();
    line2->initWithPoints(p2);

    auto line3 = std::make_shared<tt::Line>();
    line3->initWithPoints(p3);

    std::vector<std::shared_ptr<tt::Curve> > curves;
    curves.emplace_back(line2);
    curves.emplace_back(line1);
    curves.emplace_back(line3);

    remove_lines_->doRemove(curves);
    EXPECT_EQ(remove_lines_->getResult().size(), 1);

    // line 1 should be kept
    EXPECT_NEAR(remove_lines_->getResult()[0]->getLinePtr()->getD(),
                200, 1e-3*200);
}
