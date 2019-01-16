/**
 * @file QuadraticCurve.h
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date July 04, 2018
 *
 */

#ifndef LANE_LINE_QUADRATICCURVE_H
#define LANE_LINE_QUADRATICCURVE_H

#include <string>
#include <vector>

#include "lane_line/Curve.h"

namespace tt
{
/**
 * y = a + b*x + c*x*x
 */
struct QuadraticPoly
{
    float a_ = 0;
    float b_ = 0;
    float c_ = 0;
    float compute(float x) const
    {
        float res;
        res = a_ + b_*x + c_*x*x;
        return res;
    }
    std::string toString() const
    {
        std::stringstream ss;
        ss << "\n==========Quadratic Curve==========\n";
        ss << "  a + b*x + c*x*x \n";
        ss << " a: " << a_ << "\n";
        ss << " b: " << b_ << "\n";
        ss << " c: " << c_ << "\n";
        ss << "------------------------------------\n";
        return ss.str();
    }
};

class QuadraticCurve : public Curve
{
 public:
    float compute(float x) const {return poly_.compute(x);}
    QuadraticPoly& poly() {return poly_;}
    const QuadraticPoly& poly() const {return poly_;}

 public:
    /** @overload */
    int requiredPoints() const override {return 3;};

    /** @overload */
    float pointAbsDistanceToThisCurve(const Point &point) const override;

    /** @overload */
    void initWithPoints(const std::vector<Point> &points) override;

    /** @overload */
    std::shared_ptr<Curve> createCopy() const override;

    /** @overload */
    std::string toString() const override;

    /** @overload */
    std::string type() const override {return "quadratic";}

    /**
     * @overload
     *
     * we compare the coefficient of the curve.
     *
     * @param curve it must be a quadratic curve
     * @param thresholds
     * @param n     it must be 1 since we only need 1 threshold
     * @return
     */
    bool isSimilarTo(const std::shared_ptr<Curve> curve,
                     const float *thresholds,
                     const int n) const override;

    /**
     * @overload
     */
    float absDistanceToThisCurve(
            const std::shared_ptr<Curve> &other_curve) const override;
 private:
    QuadraticPoly poly_;
};

}  // namespace tt


#endif  // LANE_LINE_QUADRATICCURVE_H
