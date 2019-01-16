/**
 * @file MergeLines.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date August 02, 2018
 */

#include <vector>

#include "lane_line/Line.h"
#include "lane_line/MergeLines.h"

namespace tt
{

MergeLines::MergeLines(const MergeLinesProto &merge_lines_param,
                       const CurveFittingProto &curve_fitting_param)
{
    CHECK_GT(merge_lines_param.theta_diff_in_degree(), 0)
        << "invalid theta_diff_in_degree: "
        << merge_lines_param.theta_diff_in_degree();

    CHECK_GT(merge_lines_param.d_diff_in_pixel(), 0)
        << "invalid d_diff_in_pixel: "
        << merge_lines_param.d_diff_in_pixel();

    param_ = merge_lines_param;

    curve_fitting_ = CurveFitting::create(curve_fitting_param);
    CHECK_NOTNULL(curve_fitting_.get());
}

std::shared_ptr<MergeLines> MergeLines::create(
        const MergeLinesProto &merge_lines_param,
        const CurveFittingProto &curve_fitting_param)
{
    std::shared_ptr<MergeLines> res;
    res.reset(new MergeLines(merge_lines_param, curve_fitting_param));
    CHECK_NOTNULL(res.get());

    return res;
}

void MergeLines::doMerge(const std::vector<std::shared_ptr<Curve> > &curves)
{
    before_merge_.clear();
    after_merge_.clear();

    if (curves.empty()) return;

    // check that every curve is a line
    for (const auto &c : curves)
    {
        CHECK_EQ(c->type(), "line");
    }

    for (const auto &p : curves)
    {
        before_merge_.push_back(p->createCopy());
        after_merge_.push_back(p->createCopy());
    }

    bool anything_merged = true;
    while (anything_merged)
    {
        anything_merged = false;

        for (size_t i = 0; i < after_merge_.size(); i++)
        {
            for (size_t j = i + 1; j < after_merge_.size(); j++)
            {
                auto *line_i = after_merge_[i]->getLinePtr();
                CHECK_NOTNULL(line_i);

                auto *line_j = after_merge_[j]->getLinePtr();
                CHECK_NOTNULL(line_j);

                float diff_theta_in_degree = fabsf(line_i->getThetaInDegree()
                                                 - line_j->getThetaInDegree());

                if (diff_theta_in_degree > 180)
                {
                    diff_theta_in_degree = fabsf(360 - diff_theta_in_degree);   // todo: find a robust method  // NOLINT
                }

                if (diff_theta_in_degree > param_.theta_diff_in_degree())
                {
                    continue;
                }

                float diff_d_in_pixel = fabsf(line_i->getD() - line_j->getD());
                if (diff_d_in_pixel > param_.d_diff_in_pixel())
                {
                    continue;
                }

                LOG(INFO) << "Merge lines";
                LOG(INFO) << "line i: " << after_merge_[i]->toString();
                LOG(INFO) << "line j: " << after_merge_[j]->toString();

                // the two lines have similar theta and d,
                // then we have to merge them
                auto this_str = after_merge_[i]->toString();
                std::vector<Point> points;
                points.insert(points.end(),
                              after_merge_[i]->inliers().begin(),
                              after_merge_[i]->inliers().end());

                points.insert(points.end(),
                              after_merge_[j]->inliers().begin(),
                              after_merge_[j]->inliers().end());

                after_merge_[i] = curve_fitting_->fitCurve(points);
                LOG(WARNING) << "\n" << "this:" << this_str << "\n"
                             << "other:" << after_merge_[j]->toString() << "\n"
                             << "after merging:\n"
                             << after_merge_[i]->toString()
                             << "\n***********\n";


                after_merge_.erase(after_merge_.begin() + j);
                j--;
                anything_merged = true;
            }
        }
    }
}

}  // namespace tt
