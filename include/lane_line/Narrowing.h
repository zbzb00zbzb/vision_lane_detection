/**
 * @file Narrowing.h
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 07, 2018
 */

#ifndef LANE_LINE_NARROWING_H
#define LANE_LINE_NARROWING_H

#include <memory>

#include "lane_line/common.h"

namespace tt
{


class Narrowing
{
 public:
    explicit Narrowing(const NarrowingProto &param);
    virtual ~Narrowing() {}

    /**
     * @param image CV_8UC1
     */
    virtual void doNarrowing(const cv::Mat &image) = 0;

    /**
     * @return a binary image CV_32SC3. (leftx, 255, rightx)
     */
    virtual cv::Mat getResult() const = 0;

    NarrowingProto& getParam() {return param_;}
    const NarrowingProto& getParam() const {return param_;}

    static std::shared_ptr<Narrowing>
    create(const NarrowingProto &param);

 protected:
    NarrowingProto param_;
};


}  // namespace tt

#endif  // LANE_LINE_NARROWING_H
