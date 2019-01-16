/**
 * @file Spline.h
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date July 02, 2018
 */

#ifndef LANE_LINE_SPLINE_H
#define LANE_LINE_SPLINE_H

#include <string>
#include <vector>

#include "lane_line/Point.h"

namespace tt
{

/**
 * a spline segment.
 *
 * f(x) = a + b*(x-x0) + c*(x-x0)*(x-x0) + d*(x-x0)*(x-x0)*(x-x0)
 */
struct SplineSegment
{
    float a_ = 0;
    float b_ = 0;
    float c_ = 0;
    float d_ = 0;
    float x_ = 0;

    float step_ = 0;  //!< distance between two adjacent x, i.e., step_i = x_{i+1} - x_i     // NOLINT
    float compute(float x) const
    {
        float res;
        float diff = x - x_;

        res = a_ + b_ * diff + c_ * diff * diff + d_ * diff * diff * diff;
        return res;
    }
};

/**
 * natural cubic spline.
 *
 * Refer to the book numerical analysis, 9th edition, page 149--150
 *
 */
class Spline
{
 public:
    void compute(const std::vector<Point> &points);

    /**
     * Interpolate points.
     * @param points
     * @param num   number of points between two adjacent points.
     * @return
     */
    std::vector<Point> interpolate(const std::vector<Point> &points,
                                   int num = 20);

    std::vector<SplineSegment>& getSegments() {return segments_;}
    const std::vector<SplineSegment>& getSegments() const {return segments_;}

    std::string toString() const;

 private:
    std::vector<SplineSegment> segments_;
};


}  // namespace tt

#endif  // LANE_LINE_SPLINE_H
