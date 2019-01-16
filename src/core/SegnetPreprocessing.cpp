/**
 * @file SegnetPreprocessing.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 06, 2018
 */

#include "lane_line/SegnetPreprocessing.h"

#include "lane_line/IPMParameters.h"

#include "cnn/CnnInterface.h"

namespace tt
{

SegnetPreprocessing::SegnetPreprocessing(const PreprocessingProto &param)
    : Preprocessing(param)
{
    cnn_ = CnnInterface::create("caffe");
    cnn_->init(param.model_file(), param.trained_file(), param.device_id());
}

void SegnetPreprocessing::doPreprocessing(const cv::Mat &image)
{
    cnn_->forward(image, &probs_);
}

// width: 448, height: 448
//  2--> general
//  3--> gantry

// width: 448, height 224
// 0 background
// 1 road
// 2 lane
// 3 arrow
// 4 other
cv::Mat SegnetPreprocessing::getResult()
{
    static int i = param_.lane_type();
    return probs_[i];
}

}  // namespace tt
