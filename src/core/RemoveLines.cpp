/**
 * @file RemoveLines.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date August 02, 2018
 */

#include <algorithm>
#include <vector>

#include "lane_line/Line.h"
#include "lane_line/RemoveLines.h"

namespace tt
{

RemoveLines::RemoveLines(const RemoveLinesProto &param)
{
    CHECK_GT(param.minimum_theta_diff_in_degree(), 0);
    CHECK_GT(param.minimum_d_diff_in_pixel(), 0);

    param_ = param;
}

std::shared_ptr<RemoveLines> RemoveLines::create(
        const RemoveLinesProto &param)
{
    std::shared_ptr<RemoveLines> res;
    res.reset(new RemoveLines(param));

    CHECK_NOTNULL(res.get());
    return res;
}

void RemoveLines::doRemove(const std::vector<std::shared_ptr<Curve> > &curves)
{
    before_removal_.clear();
    after_removal_.clear();

    if (curves.empty()) return;

    // check that every curve is a line
    for (const auto &c : curves)
    {
        CHECK_EQ(c->type(), "line");
    }

    for (const auto &p : curves)
    {
        before_removal_.push_back(p->createCopy());
        after_removal_.push_back(p->createCopy());
    }

    bool anything_removed = true;
    while (anything_removed)
    {
        anything_removed = false;

        for (size_t i = 0; i < after_removal_.size(); i++)
        {
            for (size_t j = i+1; j < after_removal_.size(); j++)
            {
                auto *line_i = after_removal_[i]->getLinePtr();
                CHECK_NOTNULL(line_i);

                auto *line_j = after_removal_[j]->getLinePtr();
                CHECK_NOTNULL(line_j);

                float theta_diff = fabsf(line_i->getThetaInDegree()
                                         - line_j->getThetaInDegree());

                float d_diff = fabsf(line_i->getD() - line_j->getD());
                if (theta_diff > 180)
                {
                    theta_diff = fabsf(360 - theta_diff);   // todo: find a robust method   // NOLINT
                }
                LOG(INFO) << "theta diff: " << theta_diff;
                LOG(INFO) << "d diff: " << d_diff;

                if ((theta_diff > param_.minimum_theta_diff_in_degree()) ||
                        (d_diff < param_.minimum_d_diff_in_pixel()))
                {
                    if (after_removal_[i]->inliers().size()
                            < after_removal_[j]->inliers().size())
                    {
                        std::swap(after_removal_[i], after_removal_[j]);
                    }
                    LOG(WARNING) << "remove\n"
                                 << "keep: " << after_removal_[i]->toString()
                                 << "remove: " << after_removal_[j]->toString();

                    after_removal_.erase(after_removal_.begin() + j);
                    j--;
                    anything_removed = true;
                }
            }
        }
    }
}

}  // namespace tt

