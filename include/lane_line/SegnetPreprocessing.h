/**
 * @file SegnetPreprocessing.h
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 06, 2018
 */

#ifndef LANE_LINE_SEGNETPREPROCESSING_H
#define LANE_LINE_SEGNETPREPROCESSING_H

#include <vector>

#include "lane_line/IPMTransformation.h"
#include "lane_line/Preprocessing.h"

#include "cnn/CnnInterface.h"

namespace tt
{

class SegnetPreprocessing : public Preprocessing
{
 public:
    explicit SegnetPreprocessing(const PreprocessingProto &param);

    void doPreprocessing(const cv::Mat &image) override;
    cv::Mat getResult() override;
 private:
    std::shared_ptr<CnnInterface> cnn_;
    std::vector<cv::Mat> probs_;
};

}  // namespace tt

#endif  // LANE_LINE_SEGNETPREPROCESSING_H
