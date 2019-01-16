/**
 * @file Curve.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 08, 2018
 *
 */

#include <algorithm>
#include <string>
#include <vector>

#include "lane_line/common.h"
#include "lane_line/CubicCurve.h"
#include "lane_line/Curve.h"
#include "lane_line/Line.h"
#include "lane_line/QuadraticCurve.h"

namespace tt
{

std::shared_ptr<Curve> Curve::create(const std::string &name)
{
    std::shared_ptr<Curve> res;

    if (name == "line")
    {
        res = std::make_shared<Line>();
    }
    else if (name == "quadratic")   // NOLINT
    {
        res = std::make_shared<QuadraticCurve>();
    }
    else if (name == "cubic")   // NOLINT
    {
        res = std::make_shared<CubicCurve>();
    }
    else    // NOLINT
    {
        LOG(FATAL) << "Unknown curve name: " << name;
    }

    return res;
}

std::vector<std::shared_ptr<Curve>> Curve::sortCurves(
        const std::vector<std::shared_ptr<Curve> > &curves)
{
    std::vector<std::shared_ptr<Curve> > res(curves);

    if (curves.empty()) return res;

    if (curves[0]->type() != "line")
    {
        LOG(FATAL) << "Not implemented for " << curves[0]->type();
        return res;
    }

    std::sort(res.begin(),
              res.end(),
              [](const std::shared_ptr<tt::Curve> &first,
                 const std::shared_ptr<tt::Curve> &second)
              {
                  const auto* line1 = dynamic_cast<const tt::Line*>(first.get());       // NOLINT
                  const auto* line2 = dynamic_cast<const tt::Line*>(second.get());      // NOLINT
                  return line1->getD() < line2->getD();
              });
    return res;
}

const Line* Curve::getLinePtr() const
{
    CHECK_EQ(type(), "line");
    const Line *line = dynamic_cast<const Line*>(this);
    CHECK_NOTNULL(line);

    return line;
}

Line* Curve::getLinePtr()
{
    CHECK_EQ(type(), "line");
    Line *line = dynamic_cast<Line*>(this);
    CHECK_NOTNULL(line);

    return line;
}

float Curve::getConfidence(const float mean     /* = 100*/,
                           const float std_dev  /* = 30*/) const
{
    float n = inliers().size();

    if (n >= mean) return 1.0f / (1 << missed_num());

    CHECK_GT(fabsf(std_dev), 0) << "Invalid std_dev";

    float num = - (n - mean) * (n - mean);
    float den = 2 * std_dev * std_dev;

    float res = expf(num / den);

    res /= (1 << missed_num());

    return res;
}

std::vector<Point> Curve::getLeftBoundary() const
{
    std::vector<Point> res;
    for (const auto &p : inliers_)
    {
        res.emplace_back(p.getLeftX(), p.y());
    }

    return res;
}

std::vector<Point> Curve::getRightBoundary() const
{
    std::vector<Point> res;
    for (const auto &p : inliers_)
    {
        res.emplace_back(p.getRightX(), p.y());
    }

    return res;
}

}  // namespace tt


std::ostream& operator << (std::ostream &stream, const tt::Curve& curve)
{
    stream << curve.toString();
    return stream;
}

std::ostream&
operator << (std::ostream &stream,
             const std::vector<std::shared_ptr<tt::Curve> >& curves)
{
    for (const auto& c : curves)
    {
        stream << *c;
    }
    return stream;
}
