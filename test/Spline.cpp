/**
 * @file Spline.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date July 02, 2018
 */

#include <glog/logging.h>
#include <gtest/gtest.h>

#include <fstream>  // NOLINT
#include <vector>

#include "lane_line/Spline.h"

class SplineTest : public ::testing::Test
{
 protected:
    tt::Spline spline_;
};

/*
 * data from http://fac.ksu.edu.sa/sites/default/files/textbook-9th_edition.pdf
 * page 147
 */
TEST_F(SplineTest, case1)
{
    std::vector<tt::Point> points = {
            {1, 2}, {2, 3}, {3, 5}
    };

    spline_.compute(points);
    LOG(INFO) << spline_.toString();

    auto &seg = spline_.getSegments();
    EXPECT_EQ(seg.size(), 2);

    const float eps = 1e-6;
    EXPECT_NEAR(seg[0].a_, 2, eps);
    EXPECT_NEAR(seg[0].b_, 3.f/4, eps);
    EXPECT_NEAR(seg[0].c_, 0, eps);
    EXPECT_NEAR(seg[0].d_, 1.f/4, eps);
    EXPECT_NEAR(seg[0].x_, 1, eps);

    EXPECT_NEAR(seg[1].a_, 3, eps);
    EXPECT_NEAR(seg[1].b_, 3.f/2, eps);
    EXPECT_NEAR(seg[1].c_, 3.f/4, eps);
    EXPECT_NEAR(seg[1].d_, -1.f/4, eps);
    EXPECT_NEAR(seg[1].x_, 2, eps);
}

/*
 * data from http://fac.ksu.edu.sa/sites/default/files/textbook-9th_edition.pdf
 * page 150
 */
TEST_F(SplineTest, case2)
{
    std::vector<tt::Point> points = {
            {0, 1},
            {1, std::exp(1)},
            {2, std::exp(2)},
            {3, std::exp(3)},
    };

    spline_.compute(points);
    LOG(INFO) << spline_.toString();

    auto &seg = spline_.getSegments();
    EXPECT_EQ(seg.size(), 3);

    const float eps = 1e-3;
    EXPECT_NEAR(seg[0].a_, 1, eps);
    EXPECT_NEAR(seg[0].b_, 1.46600f, eps);
    EXPECT_NEAR(seg[0].c_, 0, eps);
    EXPECT_NEAR(seg[0].d_, 0.25228f, eps);
    EXPECT_NEAR(seg[0].x_, 0, eps);

    EXPECT_NEAR(seg[1].a_, 2.71828f, eps);
    EXPECT_NEAR(seg[1].b_, 2.22285f, eps);
    EXPECT_NEAR(seg[1].c_, 0.75685f, eps);
    EXPECT_NEAR(seg[1].d_, 1.69107f, eps);
    EXPECT_NEAR(seg[1].x_, 1, eps);

    EXPECT_NEAR(seg[2].a_, 7.38906f, eps);
    EXPECT_NEAR(seg[2].b_, 8.80977f, eps);
    EXPECT_NEAR(seg[2].c_, 5.83007f, eps);
    EXPECT_NEAR(seg[2].d_, -1.94336, eps);
    EXPECT_NEAR(seg[2].x_, 2, eps);
}

/*
 * data is from course slide http://banach.millersville.edu/~bob/math375/CubicSpline/main.pdf
 * page 8
 *
 */
TEST_F(SplineTest, case3)
{
    std::vector<tt::Point> points = {
            {5, 5},
            {7, 2},
            {9, 4},
    };

    spline_.compute(points);
    LOG(INFO) << spline_.toString();

    auto &seg = spline_.getSegments();
    EXPECT_EQ(seg.size(), 2);

    const float eps = 1e-6;
    EXPECT_NEAR(seg[0].a_, 5, eps);
    EXPECT_NEAR(seg[0].b_, -17/8.f, eps);
    EXPECT_NEAR(seg[0].c_, 0, eps);
    EXPECT_NEAR(seg[0].d_, 5/32.f, eps);
    EXPECT_NEAR(seg[0].x_, 5, eps);

    EXPECT_NEAR(seg[1].a_, 2, eps);
    EXPECT_NEAR(seg[1].b_, -1/4.f, eps);
    EXPECT_NEAR(seg[1].c_, 15/16.f, eps);
    EXPECT_NEAR(seg[1].d_, -5/32.f, eps);
    EXPECT_NEAR(seg[1].x_, 7, eps);
}

/**
 * gnuplot
 * gnuplot> plot "/tmp/a.txt" w lp
 */
TEST_F(SplineTest, case4)
{
    std::vector<tt::Point> points = {
            {0, 0},
            {0.5*M_PI, 1},
            {M_PI, 0},
            {1.5*M_PI, -1},
            {2*M_PI, 0},
    };

    auto pp = spline_.interpolate(points, 50);
    std::stringstream ss;
    for (const auto &p : pp)
    {
        ss << p.x() << " " << p.y() << "\n";
    }

    std::ofstream of("/tmp/a.txt", std::ofstream::out);
    of << ss.str();
    of.close();

    LOG(INFO) << ss.str();
}
