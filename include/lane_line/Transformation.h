/**
 * @file Transformation.h
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 04, 2018
 */

#ifndef LANE_LINE_TRANSFORMATION_H
#define LANE_LINE_TRANSFORMATION_H

#include <vector>

#include "lane_line/common.h"

namespace tt
{


class Transformation
{
 public:
    static cv::Point2f transformPoint(const cv::Matx33f &M,
                                      const cv::Point2f &p);

    static std::vector<cv::Point2f>
    transformPoints(const cv::Matx33f &M,
                    const std::vector<cv::Point2f> &points);
};


}  // namespace tt

#endif  // LANE_LINE_TRANSFORMATION_H
