/**
 * @file Line.h
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date May 30, 2018
 */

#ifndef LANE_LINE_LINE_H
#define LANE_LINE_LINE_H

#include <string>
#include <vector>

#include "lane_line/common.h"
#include "lane_line/Curve.h"

namespace tt
{


/**
 *
 * A line is represented as
 * \f[
 * x \cos \theta + y \sin \theta = d
 * \f]
 * where \f$\theta\f$ is in the range \f$[0, 2\pi)\f$
 * and \f$d \ge 0\f$.
 */
class Line : public Curve
{
 public:
    Line();
    Line(const float theta_in_radian, const float d);

    void initWithDegree(const float theta_in_degree, const float d);
    void initWithRadian(const float theta_in_radian, const float d);
    void initWithPoints(const Point &p1, const Point &p2);
    std::string type() const override {return "line";}

 public:
    /** @overload */
    int requiredPoints() const override {return 2;};
    /** @overload */
    float pointAbsDistanceToThisCurve(const Point &point) const override;
    /** @overload */
    void initWithPoints(const std::vector<Point> &points) override;

    /** @overload */
    std::shared_ptr<Curve> createCopy() const override;

    /** @overload */
    std::string toString() const override;

    /**
     * Compute the projected point on the line, i.e., compute
     * the closest point on the line to the given point.
     * @param point [in] a point
     * @return the closest point on line to the given point.
     */
    Point pointProjectedOnLine(const Point &point) const;

    /**
     * @overload
     *
     * @param curve
     * @param thresholds  0: threshold for theta (in degree), 1: threshold for d.
     * @param n
     * @return
     */
    bool isSimilarTo(const std::shared_ptr<Curve> curve,
                     const float *thresholds,
                     const int n) const override;

    /**
     * Compute the start and end points of the line in the image.
     * @param image
     * @param points 0: start point. 1: end point
     */
    void computeStartEndPoints(const cv::Mat &image,
                               std::vector<Point> &points) const;  // NOLINT


    /**
     * @overload
     */
    float absDistanceToThisCurve(
            const std::shared_ptr<Curve> &other_curve) const override;

 public:
    float getThetaInRadian() const {return theta_in_radian_;}
    float getThetaInDegree() const {return theta_in_radian_/CV_PI*180;}
    float getD() const {return d_;}
    void setThetaInRadian(float val) {theta_in_radian_ = val;}

 private:
    float theta_in_radian_;
    float d_;
};

}  // end namespace tt

#endif  // LANE_LINE_LINE_H
