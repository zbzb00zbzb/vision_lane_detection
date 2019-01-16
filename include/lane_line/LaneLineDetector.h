/**
 * @file LaneLineDetector.h
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 07, 2018
 *
 */

#ifndef LANE_LINE_LANELINEDETECTOR_H
#define LANE_LINE_LANELINEDETECTOR_H

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "lane_line/common.h"

#include "lane_line/Curve.h"
#include "lane_line/CurveFitting.h"
#include "lane_line/FilterLines.h"
#include "lane_line/IPMTransformation.h"
#include "lane_line/KalmanLineFilter.h"
#include "lane_line/MergeLines.h"
#include "lane_line/Narrowing.h"
#include "lane_line/PointsExtraction.h"
#include "lane_line/Preprocessing.h"
#include "lane_line/RemoveLines.h"

namespace tt
{

class LaneLineDetector
{
 public:
    LaneLineDetector() = default;
    virtual ~LaneLineDetector() = default;

    static std::shared_ptr<LaneLineDetector> create(
            const LaneLineDetectorProto &param);

    virtual std::string type() const;

    explicit LaneLineDetector(const LaneLineDetectorProto &param);
    void init(const LaneLineDetectorProto &param);

    void sortLines(std::vector<std::shared_ptr<tt::Curve> > *curves);

    void showDebugInfo();
    /**
     * Detect all curves in the image. No post-processing is executed.
     * @param raw_image
     * @return The detected curves.
     */
    virtual std::vector<std::shared_ptr<Curve> > detectCurves(
            const cv::Mat &raw_image);

    /**
     * Merge curves that are similar to each other.
     *
     * Due to the gap between the lane markings, points belonging to the same
     * lane may not be captured into the same point set during the points
     * extraction procedure.
     *
     * @param curves
     * @return Merged curves.
     */
    virtual std::vector<std::shared_ptr<Curve> > mergeCurves(
            const std::vector<std::shared_ptr<Curve> >& curves);

    /**
     * Fill in the missing curves.
     * @param curves
     * @return input curves + lost curves.
     */
    virtual std::vector<std::shared_ptr<Curve> > fillInMissingCurves(
            const std::vector<std::shared_ptr<Curve> >& curves);

    virtual std::vector<std::shared_ptr<Curve> > removeInvalidCurves(
            const std::vector<std::shared_ptr<Curve> >& curves);

    const LaneLineDetectorProto& param() const {return param_;}
    LaneLineDetectorProto& param() {return param_;}

    std::shared_ptr<CurveFitting> curve_fitting() const
    {return curve_fitting_;}

    std::shared_ptr<IPMTransformation> ipm_tf() const
    {return ipm_tf_;}

    std::shared_ptr<Preprocessing> preprocessing() const
    {return preprocessing_;}

    std::shared_ptr<Narrowing> narrowing() const
    {return narrowing_;}

    std::shared_ptr<PointsExtraction> points_extraction() const
    {return points_extraction_;}

    const std::deque<std::vector<std::shared_ptr<Curve> > >& queue() const
    {return detected_line_queue_;}

    std::deque<std::vector<std::shared_ptr<Curve> > >& queue()
    {return detected_line_queue_;}

    cv::Mat getIpmImage() const {return ipm_image_;}

    std::vector<std::shared_ptr<Curve> > getLeftBoundary(
            const std::vector<std::shared_ptr<Curve> >& curves) const;

    std::vector<std::shared_ptr<Curve> > getRightBoundary(
            const std::vector<std::shared_ptr<Curve> >& curves) const;

    std::shared_ptr<MergeLines>& getMergeLines() {return merge_lines_;}

    std::vector<std::shared_ptr<Curve>>& getKalmanFilteredCenterCurves()
    {
        return center_curves_;
    }

    std::vector<std::shared_ptr<Curve>>& getKalmanFilteredLeftCurves()
    {
        return left_curves_;
    }

    std::vector<std::shared_ptr<Curve>>& getKalmanFilteredRightCurves()
    {
        return right_curves_;
    }

    int getFlipImage()
    {
        return flip_image_;
    }

 protected:
    LaneLineDetectorProto param_;
    std::shared_ptr<CurveFitting> curve_fitting_;
    std::shared_ptr<IPMTransformation> ipm_tf_;
    std::shared_ptr<Preprocessing> preprocessing_;
    std::shared_ptr<Narrowing> narrowing_;
    std::shared_ptr<PointsExtraction> points_extraction_;
    std::shared_ptr<MergeLines> merge_lines_;
    std::shared_ptr<RemoveLines> remove_lines_;
    std::shared_ptr<FilterLines> filter_lines_;
    std::shared_ptr<KalmanLineFilter> kalman_center_line_filter_;
    std::shared_ptr<KalmanLineFilter> kalman_left_line_filter_;
    std::shared_ptr<KalmanLineFilter> kalman_right_line_filter_;

    std::deque<std::vector<std::shared_ptr<Curve>> > detected_line_queue_;

    cv::Mat ipm_image_;  //!< for debug purpose only

 private:
    std::vector<std::shared_ptr<Curve>> center_curves_;
    std::vector<std::shared_ptr<Curve>> left_curves_;
    std::vector<std::shared_ptr<Curve>> right_curves_;
    std::vector<cv::Mat> img_queue_;
    int img_queue_size_;
    int flip_image_;
};

}  // namespace tt


#endif  // LANE_LINE_LANELINEDETECTOR_H
