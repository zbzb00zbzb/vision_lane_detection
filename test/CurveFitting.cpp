/**
 * @file CurveFitting.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date May 30, 2018
 */

#include <gtest/gtest.h>
#include <memory>
#include <vector>

#include "lane_line/common.h"
#include "lane_line/Curve.h"
#include "lane_line/CurveFitting.h"
#include "lane_line/Drawing.h"
#include "lane_line/Line.h"


using tt::Curve;
using tt::CurveFitting;
using tt::Drawing;
using tt::Line;
using tt::Point;

class CurveFittingTest : public ::testing::Test
{
 public:
    void SetUp() override
    {
        // 3*y = 4*x + 20
        // y = (4*x+20)/3
        for (int i = 0; i < 500; i += 1)
        {
#if 1
            float x = i;
            float y = 4 * x + 20 + cv::theRNG().uniform(-5.f, 5.f);
            y /= 3;
#else
            float y = i;
            float x = 300 + cv::theRNG().uniform(-5.f, 5.f);
#endif
            points_.push_back({x, y});
        }
        points_.push_back({100, 100});
        points_.push_back({200, 100});
        points_.push_back({200, 300});
        points_.push_back({200, 400});
        points_.push_back({400, 300});

        image_ = cv::Mat::zeros(800, 600, CV_8UC3);
    }

 protected:
    std::shared_ptr<Curve> line_;
    std::vector<Point> points_;
    cv::Mat image_;
};

TEST_F(CurveFittingTest, fit_line)
{
    cv::destroyAllWindows();

    Drawing::listOfPointsInPlace(image_, points_, 1, Drawing::red(), -1);

    tt::CurveFittingProto param;
    param.set_name("line");
    param.set_threshold(10);
    param.set_max_iterations(1000);
    param.set_confidence(0.95);
    param.set_minimum_points(2);

    CurveFitting fit(param);
    line_ = fit.fitCurve(points_);

    const Line *line = dynamic_cast<Line *>(line_.get());
    Drawing::lineInPlace(image_, *line, Drawing::green(), 1);

    float theta = line->getThetaInRadian();
    float d = line->getD();
    // 3y = 4x + 20
    // -4/5x + 3/5y = 20/5
    float expected_c = -0.8f;
    float expected_s = 0.6f;
    float expected_d = 4;

    EXPECT_NEAR(expected_c, cosf(theta), 1e-3);
    EXPECT_NEAR(expected_s, sinf(theta), 1e-3);
    EXPECT_NEAR(expected_d, d, 0.1);


    cv::imshow("fit line", image_);
    cv::waitKey(0);
}
