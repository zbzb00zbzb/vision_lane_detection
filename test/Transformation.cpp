/**
 * @file Transformation.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 04, 2018
 */

#include <gtest/gtest.h>
#include <vector>

#include "lane_line/Transformation.h"

class TransformationTest : public ::testing::Test
{
 protected:
    tt::Transformation tf_;
};

TEST_F(TransformationTest, point)
{
    cv::Matx33f m = cv::Matx33f::eye();
    cv::Point2f src(1, 2);

    auto dst = tf_.transformPoint(m, src);

    EXPECT_NEAR(cv::norm(src - dst), 0, 1e-6);

    m = {1, 2, 3,
         4, 5, 6,
         7, 8, 9};

    cv::Point2f expected;
    float z = m(2, 0) * src.x + m(2, 1) * src.y + m(2, 2);
    expected.x = m(0, 0) * src.x + m(0, 1) * src.y + m(0, 2);
    expected.x /= z;

    expected.y = m(1, 0) * src.x + m(1, 1) * src.y + m(1, 2);
    expected.y /= z;

    dst = tf_.transformPoint(m, src);

    EXPECT_NEAR(cv::norm(dst - expected), 0, 1e-6);
}


TEST_F(TransformationTest, points)
{
    const float eps = 1e-6;
    cv::Matx33f m = cv::Matx33f::eye();
    std::vector<cv::Point2f> src = {{1, 2}, {3, 4}, {20, 10}};

    auto dst = tf_.transformPoints(m, src);
    for (size_t i = 0; i < src.size(); i++)
    {
        EXPECT_NEAR(cv::norm(src[i] - dst[i]), 0, eps);
    }

    m = {1, 2, 3,
         4, 5, 6,
         7, 8, 9};

    std::vector<cv::Point2f> expected;
    for (size_t i = 0; i < src.size(); i++)
    {
        cv::Point2f t;
        t.x = m(0, 0) * src[i].x + m(0, 1) * src[i].y + m(0, 2);
        t.y = m(1, 0) * src[i].x + m(1, 1) * src[i].y + m(1, 2);

        float z = m(2, 0) * src[i].x + m(2, 1) * src[i].y + m(2, 2);
        t.x /= z;
        t.y /= z;
        expected.push_back(t);
    }

    dst = tf_.transformPoints(m, src);
    for (size_t i = 0; i < src.size(); i++)
    {
        EXPECT_NEAR(cv::norm(expected[i] - dst[i]), 0, eps);
    }
}

