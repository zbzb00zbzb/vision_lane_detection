/**
 * @file Line.h
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date May 31, 2018
 */

#ifndef LANE_LINE_CURVE_H
#define LANE_LINE_CURVE_H


#include <memory>
#include <string>
#include <vector>

#include "lane_line/Point.h"

namespace tt
{

class Line;

/**
 * An abstract class for curves.
 */
class Curve
{
 public:
    Curve() = default;
    virtual ~Curve() {}

    static std::shared_ptr<Curve> create(const std::string &name);

    static std::vector<std::shared_ptr<Curve> > sortCurves(
            const std::vector<std::shared_ptr<Curve> > &curves);

    const Line *getLinePtr() const;
    Line *getLinePtr();
 public:
    /**
     * If the number of inliers is greater than mean, then return 1.
     * Ohterwise, return
     * \f[
     *     exp(-\frac{(n - mean)*(n - mean)}{2*std_dev*std_dev})
     * \f]
     *
     * where n is the number of inliners.
     *
     *
     * @param mean    mean value
     * @param std_dev standard deviation
     * @return
     */
    float getConfidence(const float mean = 100,
                        const float std_dev = 30) const;

    std::vector<Point> getLeftBoundary() const;
    std::vector<Point> getRightBoundary() const;

 public:
    virtual std::string type() const = 0;
    /**
     * The number of required points to determine the curve.
     * @return
     */
    virtual int requiredPoints() const = 0;
    /**
     * Init this curve from a list of points.
     * @param points A list of the points on the curve.
     */
    virtual void initWithPoints(const std::vector<Point> &points) = 0;

    /**
     * Compute the distance from a point to the curve.
     * @param point A given point.
     * @return Distance from the point to the curve.
     */
    virtual float pointAbsDistanceToThisCurve(const Point &point) const = 0;

    virtual float
    absDistanceToThisCurve(const std::shared_ptr<Curve> &other_curve) const = 0;

    /**
     * Create a copy of the curve.
     * @return A copy of the curve.
     */
    virtual std::shared_ptr<Curve> createCopy() const = 0;

    /**
     *  Return if this curve is similar to the given curve under specified thresholds.
     *
     * @param curve         another curve
     * @param thresholds    a list of thresholds
     * @param n             the number of elements in thresholds
     * @return
     */
    virtual bool isSimilarTo(const std::shared_ptr<Curve> curve,
                             const float *thresholds,
                             const int n) const = 0;
    virtual std::string toString() const = 0;

 public:
    std::vector<Point> &inliers() {return inliers_;}
    const std::vector<Point> &inliers() const {return inliers_;}

    const int missed_num() const {return missed_num_;}
    int& missed_num() {return missed_num_;}

 protected:
    std::vector<Point> inliers_;  //!< the points used to compute the curve.
    int missed_num_ = 0;          //!< number of times this curve has been missing      // NOLINT
};

}  // end namespace tt

std::ostream& operator << (std::ostream &stream, const tt::Curve& curve);

std::ostream&
operator << (std::ostream &stream,
             const std::vector<std::shared_ptr<tt::Curve> >& curves);

#endif  // LANE_LINE_CURVE_H
