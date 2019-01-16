/**
 * @file Transformation.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 04, 2018
 */

#include "lane_line/Transformation.h"

#include <vector>

namespace tt
{


cv::Point2f Transformation::transformPoint(const cv::Matx33f &M,
                                           const cv::Point2f &p)
{
    cv::Vec3f v(p.x, p.y, 1);
    cv::Vec3f mv = M * v;

    cv::Point2f res;
    res.x = mv[0] / mv[2];
    res.y = mv[1] / mv[2];

    return res;
}

std::vector<cv::Point2f>
Transformation::transformPoints(const cv::Matx33f &M,
                                const std::vector<cv::Point2f> &points)
{
    std::vector<cv::Point2f> res;
    res.reserve(points.size());
    for (const auto &p : points)
    {
        res.push_back(transformPoint(M, p));
    }

    return res;
}


}  // namespace tt
