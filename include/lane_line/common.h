/**
 * @file common.h
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date May 30, 2018
 */

#ifndef LANE_LINE_COMMON_H
#define LANE_LINE_COMMON_H

#include <glog/logging.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <string>

#include "proto/lane_line.pb.h"

namespace tt
{

void loadConfigFromFile(const std::string &filename,
                        LaneLineDetectorProto *param);

void saveConfigToFile(const std::string &filename,
                      const LaneLineDetectorProto &param);

void checkConfig(const LaneLineDetectorProto &param);

}  // namespace tt

#endif  // LANE_LINE_COMMON_H
