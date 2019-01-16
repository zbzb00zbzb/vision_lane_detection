/**
 * @file RemoveLines.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date August 02, 2018
 */

#ifndef LANE_LINE_REMOVELINES_H
#define LANE_LINE_REMOVELINES_H

#include <memory>
#include <vector>

#include "lane_line/common.h"
#include "lane_line/Curve.h"

namespace tt
{

/**
 * If the theta_diff between two lines is larger than
 * the given value, the line with smaller number of inliers is removed.
 *
 * In addition, if the distance between two lines is less than
 * the given threshold, the the line with smaller number of inliers is removed.
 */
class RemoveLines
{
 public:
    explicit RemoveLines(const RemoveLinesProto &param);
    static std::shared_ptr<RemoveLines> create(const RemoveLinesProto &param);

    const RemoveLinesProto& param() const {return param_;}
    RemoveLinesProto& param() {return param_;}

    void doRemove(const std::vector<std::shared_ptr<Curve> > &curves);

    const std::vector<std::shared_ptr<Curve> >& getResult() const
    {
        return after_removal_;
    }

    std::vector<std::shared_ptr<Curve> >& getResult()
    {
        return after_removal_;
    }
 private:
    RemoveLinesProto param_;
    std::vector<std::shared_ptr<Curve> > before_removal_;
    std::vector<std::shared_ptr<Curve> > after_removal_;
};

}  // namespace tt

#endif  // LANE_LINE_REMOVELINES_H
