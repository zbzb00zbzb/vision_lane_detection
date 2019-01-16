/**
 * @file MergeLines.h
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date August 02, 2018
 *
 */
#ifndef LANE_LINE_MERGELINES_H
#define LANE_LINE_MERGELINES_H

#include <memory>
#include <vector>

#include "lane_line/common.h"
#include "lane_line/Curve.h"
#include "lane_line/CurveFitting.h"

namespace tt
{

/**
 * It merges two lines if their theta and d
 * are within the specified thresholds.
 */
class MergeLines
{
 public:
    MergeLines(const MergeLinesProto &merge_lines_param,
               const CurveFittingProto &curve_fitting_param);

    static std::shared_ptr<MergeLines> create(
            const MergeLinesProto &merge_lines_param,
            const CurveFittingProto &curve_fitting_param);

    const MergeLinesProto& param() const {return param_;}
    MergeLinesProto& param() {return param_;}

    void doMerge(const std::vector<std::shared_ptr<Curve> > &curves);

    const std::vector<std::shared_ptr<Curve> >& getResult() const
    {
        return after_merge_;
    }

    std::vector<std::shared_ptr<Curve> >& getResult()
    {
        return after_merge_;
    }

    std::shared_ptr<CurveFitting> getCurveFitting() const
    {
        return curve_fitting_;
    }

 private:
    MergeLinesProto param_;
    std::vector<std::shared_ptr<Curve> > before_merge_;
    std::vector<std::shared_ptr<Curve> > after_merge_;

    std::shared_ptr<CurveFitting> curve_fitting_;
};

}  // namespace tt

#endif  // LANE_LINE_MERGELINES_H
