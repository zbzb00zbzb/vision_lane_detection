/**
 * @file test_common.h
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date August 02, 2018
 *
 * @brief common functions used only inside the test.
 */

#ifndef LANE_LINE_TEST_COMMON_H
#define LANE_LINE_TEST_COMMON_H

#include <vector>

#include "lane_line/Point.h"

namespace tt
{

/**
 * Generate test line data.
 * @param theta_in_radian
 * @param d
 * @param start_x
 *
 * @return a list of points belonging to the same line determined
 * by theta_in_radian and d.
 */
std::vector<tt::Point> generatePoints(
        float theta_in_radian,
        float d,
        float start_x);


}  // namespace tt

#endif  // LANE_LINE_TEST_COMMON_H
