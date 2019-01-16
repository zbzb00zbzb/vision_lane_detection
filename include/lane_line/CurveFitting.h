/**
 * @file CurveFitting.h
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date May 31, 2018
 */

#ifndef LANE_LINE_CURVEFITTING_H
#define LANE_LINE_CURVEFITTING_H

#include <memory>
#include <vector>

namespace tt
{

class Curve;
class Point;

class CurveFitting
{
 public:
    explicit CurveFitting(const CurveFittingProto &param);
    static std::shared_ptr<CurveFitting> create(const CurveFittingProto &param);

    std::shared_ptr<Curve> fitCurve(const std::vector<Point> &points) const;

    std::vector<std::shared_ptr<Curve> >
    fitCurves(const std::vector<std::vector<Point> > &points) const;

 public:
    CurveFittingProto& param() {return param_;}
    const CurveFittingProto& param() const {return param_;}

 private:
    CurveFittingProto param_;
};

}  // end namespace tt
#endif  // LANE_LINE_CURVEFITTING_H
