/**
 * @file Preprocessing.h
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 06, 2018
 */

#ifndef LANE_LINE_PREPROCESSING_H
#define LANE_LINE_PREPROCESSING_H

#include <memory>

#include "lane_line/common.h"

namespace tt
{


class Preprocessing
{
 public:
    explicit Preprocessing(const PreprocessingProto &param);
    virtual ~Preprocessing() {}

    virtual void doPreprocessing(const cv::Mat &image) = 0;
    virtual cv::Mat getResult() = 0;

    PreprocessingProto& getParam() {return param_;}
    const PreprocessingProto& getParam() const {return param_;}

    static std::shared_ptr<Preprocessing>
    create(const PreprocessingProto &param);

 protected:
    PreprocessingProto param_;
};


}  // namespace tt

#endif  // LANE_LINE_PREPROCESSING_H
