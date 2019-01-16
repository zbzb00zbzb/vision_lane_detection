/**
 * @file QuadraticCurve.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date July 04, 2018
 *
 */

#include <string>
#include <vector>

#include "lane_line/common.h"

#include "lane_line/QuadraticCurve.h"

namespace tt
{

// we do not compute the exact distance here!
float QuadraticCurve::pointAbsDistanceToThisCurve(const Point &point) const
{
    float x = point.x();
    float y;
    y = poly_.compute(x);
    return fabsf(y - point.y());
}

void QuadraticCurve::initWithPoints(const std::vector<Point> &points)
{
    CHECK_GE(points.size(), requiredPoints());

    cv::Mat A;
    cv::Mat y;

    int n = static_cast<int>(points.size());

    A.create(n, 3, CV_32FC1);
    y.create(n, 1, CV_32FC1);

    for (int i = 0; i < n; i++)
    {
        auto *pa = A.ptr<float>(i);
        auto *py = y.ptr<float>(i);
        const auto &pp = points[i];

        pa[0] = 1;
        pa[1] = pp.x();
        pa[2] = pp.x() * pp.x();

        py[0] = pp.y();
    }

    cv::Vec3f x;
    cv::solve(A, y, x, cv::DecompTypes::DECOMP_QR);
    poly_.a_ = x[0];
    poly_.b_ = x[1];
    poly_.c_ = x[2];

    inliers() = points;
}

std::shared_ptr<Curve> QuadraticCurve::createCopy() const
{
    auto result = std::make_shared<QuadraticCurve>();

    result->inliers() = inliers();
    result->missed_num() = missed_num();
    result->poly_ = poly_;

    return result;
}

std::string QuadraticCurve::toString() const
{
    std::stringstream ss;
    ss << poly_.toString();
    ss << "inliers count: " << inliers_.size() << "\n";
    return ss.str();
}

bool QuadraticCurve::isSimilarTo(const std::shared_ptr<Curve> curve,
                                 const float *thresholds,
                                 const int n) const
{
    CHECK_NOTNULL(curve.get());
    CHECK_EQ(curve->type(), this->type());

    CHECK_NOTNULL(thresholds);
    CHECK_EQ(n, 1);

    auto *other = dynamic_cast<QuadraticCurve*>(curve.get());
    CHECK_NOTNULL(other);

    float diff = 0;

    diff += std::fabs(other->poly().a_ - this->poly_.a_);
    diff += std::fabs(other->poly().b_ - this->poly_.b_);
    diff += std::fabs(other->poly().c_ - this->poly_.c_);

    return diff < thresholds[0];
}

float QuadraticCurve::absDistanceToThisCurve(
        const std::shared_ptr<Curve> &other_curve) const
{
    LOG(FATAL) << "Not implemented yet!";
    return 100;
}

}  // namespace tt
