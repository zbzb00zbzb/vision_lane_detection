/**
 * @file FilterLines.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date August 02, 2018
 *
 */

#include <vector>

#include "lane_line/Curve.h"
#include "lane_line/FilterLines.h"
#include "lane_line/Line.h"

namespace
{
/**
 *
 * @param curve the current curve
 * @param curves the history
 * @return true if curve appears in the history of curves, false otherwise.
 */
bool wasDetected(const std::shared_ptr<tt::Curve> &curve,
                 const std::vector<std::shared_ptr<tt::Curve> > &curves,
                 const tt::FilterLinesProto &param)
{
    bool res = false;

    auto *current_line = curve->getLinePtr();
    CHECK_NOTNULL(current_line);

    for (const auto &c : curves)
    {
        auto *history_line = c->getLinePtr();
        CHECK_NOTNULL(history_line);

        float theta_diff = fabsf(history_line->getThetaInDegree()
                                 - current_line->getThetaInDegree());
        if (theta_diff > 180)
        {
            theta_diff = fabsf(360 - theta_diff);
        }

        if (theta_diff > param.minimum_theta_diff_in_degree())
        {
            continue;
        }

        float d_diff = fabsf(history_line->getD() - current_line->getD());
        if (d_diff > param.minimum_d_diff_in_pixel())
        {
            continue;
        }

        res = true;
        break;
    }

    return res;
}

}  // namespace

namespace tt
{

FilterLines::FilterLines(const FilterLinesProto &param)
{
    CHECK_GT(param.queue_size(), 0);
    CHECK_GT(param.minimum_detected_count(), 0);
    CHECK_LE(param.minimum_detected_count(), param.queue_size());

    CHECK_GT(param.minimum_theta_diff_in_degree(), 0);
    CHECK_GT(param.minimum_d_diff_in_pixel(), 0);
    param_ = param;
}

std::shared_ptr<FilterLines> FilterLines::create(const FilterLinesProto &param)
{
    std::shared_ptr<FilterLines> res;
    res.reset(new FilterLines(param));
    CHECK_NOTNULL(res.get());

    return res;
}

std::vector<std::shared_ptr<tt::Curve> > FilterLines::filterLines(
        const std::vector<std::shared_ptr<tt::Curve> > &curves)
{
    for (const auto &c : curves)
    {
        CHECK_EQ(c->type(), "line");
    }

    std::vector<std::shared_ptr<Curve> > res;
    for (const auto &p : curves)
    {
        if (p->inliers().size() > param_.at_least_inlier_count())
        {
            res.push_back(p->createCopy());
            continue;
        }

        int detected_count = 0;
        for (size_t i = 0 ; i < detected_lines_queue_.size(); i++)
        {
            detected_count += wasDetected(p, detected_lines_queue_[i], param_);
        }
        if (detected_count >= param_.minimum_detected_count())
        {
            res.push_back(p->createCopy());
        }
    }

    detected_lines_queue_.push_back(curves);
    if (detected_lines_queue_.size() > param_.queue_size())
    {
        detected_lines_queue_.pop_front();
    }

    return res;
}

}  // namespace tt
