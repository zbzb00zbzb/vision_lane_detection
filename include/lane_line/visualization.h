/**
 * @file visualization.h
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date August 13, 2018
 */

#ifndef LANE_LINE_VISUALIZATION_H
#define LANE_LINE_VISUALIZATION_H

#include <vector>

#include "lane_line/common.h"
#include "lane_line/config.h"

#include "lane_line/Curve.h"
#include "lane_line/Drawing.h"
#include "lane_line/FileSystem.h"
#include "lane_line/LaneLineDetector.h"

#include "lane_line/protobuf_util.h"


namespace tt
{

cv::Mat visualizeResult(std::shared_ptr <tt::LaneLineDetector> detector,
                        const cv::Mat &raw_image,
                        bool draw_left_right_boundary,
                        bool only_show_original_image);

}  // namespace tt

#endif  // LANE_LINE_VISUALIZATION_H
