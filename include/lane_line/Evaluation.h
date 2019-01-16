/**
 * @file Evaluation.h
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date August 16, 2018
 */

#ifndef LANE_LINE_EVALUATION_H
#define LANE_LINE_EVALUATION_H

#include <opencv2/core.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "lane_line/common.h"
#include "lane_line/config.h"

#include "lane_line/Curve.h"
#include "lane_line/CurveFitting.h"
#include "lane_line/IPMTransformation.h"
#include "lane_line/MergeLines.h"
#include "lane_line/Narrowing.h"
#include "lane_line/PointsExtraction.h"
#include "lane_line/RemoveLines.h"

namespace tt
{

/**
 *
 */
class Evaluation
{
 public:
    explicit Evaluation(const EvaluationProto &param);
    static std::shared_ptr<Evaluation> create(const EvaluationProto &param);

 public:
    const EvaluationProto& param() const { return param_; }
    EvaluationProto& param() { return param_; }

    void generateListOfMasks();
    void generateIpmMasks();
    void generateLines();

    const std::unordered_map<std::string, cv::Mat>& getIpmMasks() const
    { return ipm_masks_; }

    std::unordered_map<std::string, cv::Mat>& getIpmMasks()
    { return ipm_masks_; }
 private:
    /**
     * Check that the passed param is valid.
     * @param param
     * @param error_str [out] the error info is return in this variable
     * @return true if it valid, false otherwise
     */
    bool checkParam(const EvaluationProto &param, std::string *error_str) const;
 private:
    std::unordered_map<std::string, cv::Mat> ipm_masks_;
    std::vector<std::string> mask_filename_;
    EvaluationProto param_;
    std::shared_ptr<IPMTransformation> ipm_tf_;

    std::shared_ptr<Narrowing> narrowing_;
    std::shared_ptr<PointsExtraction> points_extraction_;
    std::shared_ptr<CurveFitting> curve_fitting_;
    std::shared_ptr<MergeLines> merge_lines_;
    std::shared_ptr<RemoveLines> remove_lines_;
};

}  // namespace tt


#endif  // LANE_LINE_EVALUATION_H
