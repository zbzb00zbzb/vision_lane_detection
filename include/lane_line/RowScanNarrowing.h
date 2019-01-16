/**
 * @file RowScanNarrowing.h
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 07, 2018
 */

#ifndef LANE_LINE_ROWSCANNARROWING_H
#define LANE_LINE_ROWSCANNARROWING_H

#include "lane_line/Narrowing.h"

namespace tt
{

/**
 * Change a connected area to a line area.
 * Only 1 point remains for each row inside the area.
 *
 * If the row width inside the area is less than minimum_width_
 * or the pixel intensity inside the area is less than threshold_
 * then this row is ignore.
 *
 */
class RowScanNarrowing : public Narrowing
{
 public:
    explicit RowScanNarrowing(const NarrowingProto & param);

    void doNarrowing(const cv::Mat &image) override;
    /**
     *
     * @return CV_32SC3,  channel 0: the index of the left boundary of the area
     *                    channel 1: 255 if the area passes the test
     *                    channel 2: the index of the right boundary of the area
     */
    cv::Mat getResult() const override;

 private:
    cv::Mat result_;
    float threshold_;  //!< values less than this value are ignored.
    int minimum_width_;
};


}  // namespace tt

#endif  // LANE_LINE_ROWSCANNARROWING_H

