/**
 * @file KalmanLineFilter.h
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date August 08, 2018
 *
 */

#ifndef LANE_LINE_KALMANLINEFILTER_H
#define LANE_LINE_KALMANLINEFILTER_H

#include <opencv2/video.hpp>

#include <memory>
#include <unordered_map>
#include <vector>

#include "lane_line/common.h"
#include "lane_line/Curve.h"

namespace tt
{
/**
 * Filter the detected lines with kalman filter.
 * Usage:
 *  1. create an instance `kalman'.
 *  2. kalman->doKalmanLineFilter(curves)  to filter the lines
 *
 * The tracked state of a line is
 *  - theta in degree
 *  - d in pixel
 *
 *  We assume that theta in degree is near 0 degree. That is,
 *  the driving direction of the vehicle is parallel with the lane marking.
 */

class KalmanLineFilter
{
 public:
    explicit KalmanLineFilter(const KalmanLineFilterProto &param);
    static std::shared_ptr<KalmanLineFilter> create(
        const KalmanLineFilterProto &param);

    /**
     * Filter the detected curves.
     * @param curves
     */
    void doKalmanLineFilter(std::vector<std::shared_ptr<Curve>> *curves);
 private:
    /**
     * Remove lines if they have not been detected for specified counts.
     */
    void removeLines();

    /**
     * If the line was detected, return a
     * @param line
     * @return
     */
    int wasDetected(const Line &line) const;

    int getNextAvailableId() const;

    /**
     * We need to track a newly detected line.
     * @param line
     */
    void createTrackedLines(const Line &line);

    void predict(int id);
    void update(int id, const Line &line);
    /**
     * Return the posterior state.
     * @param id
     * @param line
     */
    void getState(int id, Line *line);

 private:
    struct TrackedLines
    {
        int missed_count_ = 0;
        cv::KalmanFilter kalman_filter_;
    };
 private:
    KalmanLineFilterProto param_;

    /**
     * key: int, a non-negative integer
     * TrackedLines: tracked lines
     */
    std::unordered_map<int, TrackedLines> detected_lines_;
};

}  // namespace tt

#endif  // LANE_LINE_KALMANLINEFILTER_H
