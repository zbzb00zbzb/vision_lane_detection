/**
 * @file CurveFitting.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date May 31, 2018
 */

#include <algorithm>
#include <iostream>     // NOLINT
#include <limits>
#include <set>
#include <vector>

#include "lane_line/common.h"
#include "lane_line/Curve.h"
#include "lane_line/CurveFitting.h"
#include "lane_line/Point.h"

namespace
{
int getRansacExpectedIterations(float confidence, float outlier_ratio,
                                int num_model_points, int max_iterations)
{
    confidence = std::max(confidence, 0.f);
    confidence = std::min(confidence, 1.f);
    outlier_ratio = std::max(outlier_ratio, 0.f);
    outlier_ratio = std::min(outlier_ratio, 1.f);

    // avoid inf's & nan's
    float num = std::max(1.f - confidence, std::numeric_limits<float>::min());
    float denom = 1.f - std::pow(1.f - outlier_ratio,
                                 static_cast<float>(num_model_points));
    if (denom < std::numeric_limits<float>::min())
    {
        return 0;
    }

    num = std::log(num);
    denom = std::log(denom);

    return denom >= 0 || -num >= max_iterations*(-denom) ? max_iterations
                                                         : cvRound(num/denom);
}

using tt::Point;
void getSampledPoints(const std::vector<Point> &input_points,
                      const int n,
                      std::vector<Point> &output_points) // NOLINT
{
    CHECK_GT(static_cast<int>(input_points.size()), n)
        << "Not enough number of points!";
    output_points.clear();

    int num_available_points = static_cast<int>(input_points.size());

    std::set<int> indices;
    while (static_cast<int>(indices.size()) < n)
    {
        int id = cv::theRNG().uniform(static_cast<int>(0),
                                      num_available_points);
        if (indices.count(id)) continue;
        bool is_ok = true;
        for (const auto i : indices)
        {
            const cv::Point2f this_point = input_points[i];
            const cv::Point2f other_point = input_points[id];

            if (cv::norm(this_point - other_point) < 5)
            {
                is_ok = false;
                break;
            }
        }
        if (!is_ok) continue;  // TODO (fangjun): this may be an infinite loop!

        indices.insert(id);
    }

    for (auto id : indices)
    {
        output_points.push_back(input_points[id]);
    }
}

}  // end unnamed namespace

namespace
{

}

namespace tt
{

CurveFitting::CurveFitting(const CurveFittingProto &param)
    : param_(param)
{
    CHECK_GT(param_.confidence(), 0);
    CHECK_LT(param_.confidence(), 1);

    CHECK_GT(param_.max_iterations(), 0);

    CHECK_GT(param_.minimum_points(), 0);
}

std::shared_ptr<CurveFitting>
CurveFitting::create(const CurveFittingProto &param)
{
    auto res = std::make_shared<CurveFitting>(param);

    return res;
}

std::vector<std::shared_ptr<Curve> >
CurveFitting::fitCurves(const std::vector<std::vector<Point> > &points) const
{
    std::vector<std::shared_ptr<Curve> > curves;
    for (const auto &p : points)
    {
        if (p.size() < param_.minimum_points()) continue;

        auto c = fitCurve(p);
        curves.push_back(c);
    }

    return curves;
}

std::shared_ptr<Curve>
CurveFitting::fitCurve(const std::vector<Point> &points) const
{
    auto curve = Curve::create(param_.name());
    CHECK_NOTNULL(curve.get());

    int num_points = static_cast<int>(points.size());
    CHECK_GE(num_points, curve->requiredPoints())
        << "Not enough number of points!";

    std::shared_ptr<Curve> best_curve;
    std::vector<Point> current_points;
    int best_inliers_num = -1;

    int model_points = curve->requiredPoints();

    int max_iterations = param_.max_iterations();
    for (int i = 0; i < max_iterations; i++)
    {
        getSampledPoints(points, model_points, current_points);
        curve->initWithPoints(current_points);

        int inliers_num = 0;
        for (const auto &p : points)
        {
            float dist = curve->pointAbsDistanceToThisCurve(p);
            if (dist <= param_.threshold()) inliers_num++;
        }

        if (inliers_num > best_inliers_num)
        {
            best_inliers_num = inliers_num;
            best_curve = curve->createCopy();
        }

        float outlier_ratio = 1.0f -
                best_inliers_num / static_cast<float>(num_points);
        max_iterations = getRansacExpectedIterations(param_.confidence(),
                                                     outlier_ratio,
                                                     model_points,
                                                     max_iterations);
    }

    // refinement
    curve = best_curve;
    std::vector<Point> inliers;
    for (const auto &p : points)
    {
        float dist = curve->pointAbsDistanceToThisCurve(p);
        if (dist <= param_.threshold()) inliers.push_back(p);
    }
    curve->initWithPoints(inliers);
    return curve;
}

}  // end namespace tt
