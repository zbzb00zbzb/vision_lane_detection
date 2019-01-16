/**
 * @file GantryLaneDetector.h
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 25, 2018
 */

#ifndef LANE_LINE_GANTRYLANEDETECTOR_H
#define LANE_LINE_GANTRYLANEDETECTOR_H

#include <string>
#include <vector>

#include "lane_line/LaneLineDetector.h"

namespace tt
{

class GantryLaneDetector : public LaneLineDetector
{
 public:
    explicit GantryLaneDetector(const LaneLineDetectorProto& param);

    std::string type() const override {return "gantry";}

    std::vector<std::shared_ptr<Curve> > detectCurves(
            const cv::Mat &raw_image) override;
 private:
    std::vector<std::shared_ptr<Curve> > filtering(
            const std::vector<std::shared_ptr<Curve> >& curves);

    std::vector<std::shared_ptr<Curve> > fixDirections(
            const std::vector<std::shared_ptr<Curve> >& curves);
};

}  // namespace tt

#endif  // LANE_LINE_GANTRYLANEDETECTOR_H
