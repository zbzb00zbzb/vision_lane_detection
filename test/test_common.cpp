/**
 * @file test_common.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date August 02, 2018
 *
 * @brief common functions used only inside the test.
 */

#include <vector>

#include "lane_line/Point.h"

namespace tt
{

// generate test data

std::vector<tt::Point> generatePoints(
        float theta_in_radian,
        float d,
        float start_x)
{
    // x*cos(theta) + y*sin(theta) = d
    int n = 20;
    std::vector<tt::Point> points;

    if (fabsf(sinf(theta_in_radian)) < 1e-2)
    {
        float x;
        if (cosf(theta_in_radian) > 0)
        {
            x = d;
        }
        else    // NOLINT
        {
            x = -d;
        }

        // a vertical line
        for (int i = 0; i < n; i++)
        {
            float y = cv::theRNG().uniform(10, 300);
            points.emplace_back(x, y);
        }
    }
    else        // NOLINT
    {
        // not a vertical line
        float c = cosf(theta_in_radian);
        float s = sinf(theta_in_radian);
        float x = start_x;
        for (int i = 0; i < n; i++)
        {
            // x*cos(theta) + y*sin(theta) = d;
            float y = (d - x * c)/s;
            points.emplace_back(x, y);
            x += 10;
        }
    }

    return points;
}

}  // namespace tt

