/**
 * @file FilterLines.h
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date August 02, 2018
 *
 */
#ifndef LANE_LINE_FILTERLINES_H
#define LANE_LINE_FILTERLINES_H

#include <deque>
#include <memory>
#include <vector>

#include "lane_line/common.h"
#include "lane_line/Curve.h"

namespace tt
{

/**
 * It keeps a queue of detected lines.
 * Only if a line has been detected at least minimum_detected_count
 * can it be shown to the user. Otherwise, it is kept in the queue.
 */
class FilterLines
{
 public:
    explicit FilterLines(const FilterLinesProto &param);
    static std::shared_ptr<FilterLines> create(const FilterLinesProto &param);

    /**
     *
     * @param curves the detected curves in the current image.
     * @return Remove the lines inside curves that has not been detected for
     * minimum_detected_count
     */
    std::vector<std::shared_ptr<tt::Curve> > filterLines(
            const std::vector<std::shared_ptr<tt::Curve> > &curves);

 private:
    std::deque<std::vector<std::shared_ptr<Curve>> > detected_lines_queue_;
    FilterLinesProto param_;    // minimum_detected_count is contained in param_
};

}  // namespace tt

#endif  // LANE_LINE_FILTERLINES_H
