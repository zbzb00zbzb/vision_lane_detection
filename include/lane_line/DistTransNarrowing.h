/**
 * @file DistTransNarrowing.h
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 07, 2018
 *
 * @brief Distance transform narrowing.
 */

#ifndef LANE_LINE_DISTTRANSNARROWING_H
#define LANE_LINE_DISTTRANSNARROWING_H

#include "lane_line/Narrowing.h"

namespace tt
{


class DistTransNarrowing : public Narrowing
{
 public:
    explicit DistTransNarrowing(const NarrowingProto & param);

    void doNarrowing(const cv::Mat &image) override;
    cv::Mat getResult() const override;

 private:
    cv::Mat result_;
    float intensity_thresold_;
    float dist_thresold_;
};


}  // namespace tt

#endif  // LANE_LINE_DISTTRANSNARROWING_H
