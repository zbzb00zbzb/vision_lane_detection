/**
 * @file PointsExtraction.h
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 07, 2018
 *
 */

#ifndef LANE_LINE_POINTSEXTRACTION_H
#define LANE_LINE_POINTSEXTRACTION_H

#include <memory>
#include <vector>

#include "lane_line/common.h"

#include "lane_line/Point.h"

namespace tt
{


class PointsExtraction
{
 public:
    explicit PointsExtraction(const PointsExtractionProto &param);

    /**
     *
     * @param image CV_32SC3
     */
    void doExtraction(const cv::Mat &image);
    void doExtraction_slide_windows(const cv::Mat &image);

    std::vector<std::vector<Point> >& getPoints() {return points_; }
    const std::vector<std::vector<Point> >& getPoints() const {return points_;}

    static std::shared_ptr<PointsExtraction>
    create(const PointsExtractionProto &param);

 private:
    PointsExtractionProto param_;
    std::vector<std::vector<Point> > points_;

    int max_x_dir_search_;
    int max_y_dir_search_;
};


}  // namespace tt

#endif  // LANE_LINE_POINTSEXTRACTION_H
