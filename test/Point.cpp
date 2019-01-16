/**
 * @file Point.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date May 30, 2018
 */

#include <gtest/gtest.h>
#include <memory>

#include "lane_line/Point.h"

using tt::Point;

class PointTest : public ::testing::Test
{
 public:
    void SetUp() override
    {
        p_ = std::make_shared<Point>();
    }

 protected:
    std::shared_ptr<Point> p_;
    const float eps_ = 1e-6;
};

TEST_F(PointTest, empty_constructor)
{
    EXPECT_NEAR(p_->getX(), -1, eps_);
    EXPECT_NEAR(p_->getY(), -1, eps_);
    EXPECT_NEAR(p_->getLeftX(), -1, eps_);
    EXPECT_NEAR(p_->getRightX(), -1, eps_);
}

TEST_F(PointTest, conversion)
{
    p_.reset(new Point(1, 2));

    cv::Point2f expected(1, 2);
    cv::Point2f actual(*p_);

    EXPECT_NEAR(cv::norm(actual - expected), 0, eps_);
}

TEST_F(PointTest, setter_getter)
{
    p_.reset(new Point());

    p_->setXY(3, 4);
    p_->setLeftX(1);
    p_->setRightX(5);

    EXPECT_NEAR(p_->getX(), 3, eps_);
    EXPECT_NEAR(p_->getY(), 4, eps_);
    EXPECT_NEAR(p_->getLeftX(), 1, eps_);
    EXPECT_NEAR(p_->getRightX(), 5, eps_);

    cv::Point2f expected(3, 4);
    cv::Point2f actual = p_->getXY();

    EXPECT_NEAR(cv::norm(actual - expected), 0, eps_);

    expected = cv::Point2f(1, 4);
    actual = p_->getLeftXY();
    EXPECT_NEAR(cv::norm(actual - expected), 0, eps_);

    expected = cv::Point2f(5, 4);
    actual = p_->getRightXY();
    EXPECT_NEAR(cv::norm(actual - expected), 0, eps_);
}

TEST_F(PointTest, to_string)
{
    p_->setXY(10, 20);
    p_->setLeftX(3);
    p_->setRightX(15);

    std::cout << *p_;
}
