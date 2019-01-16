/**
 * @file DistTransNarrowing.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 07, 2018
 *
 */

#include "lane_line/DistTransNarrowing.h"

namespace tt
{


DistTransNarrowing::DistTransNarrowing(const NarrowingProto &param)
    : Narrowing(param),
      intensity_thresold_(param.dist_trans_param().intensity_threshold()),
      dist_thresold_(param.dist_trans_param().dist_threshold())
{}

void DistTransNarrowing::doNarrowing(const cv::Mat &image)
{
    cv::Mat dist;
    cv::Mat b;
    b = image > intensity_thresold_;
    cv::distanceTransform(image, dist, cv::DIST_L1, 3, CV_8U);
    int width = image.cols;
    int height = image.rows;

    for (int y = 0; y < height; y++)
    {
        auto p = dist.ptr<uchar>(y);
        for (int x = 0; x < width; x++)
        {
            if (p[x] < dist_thresold_)
            {
                p[x] = 0;
            }
        }
    }
    result_ = dist;
}

cv::Mat DistTransNarrowing::getResult() const
{
    return result_;
}


}  // namespace tt
