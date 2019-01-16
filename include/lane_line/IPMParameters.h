/**
 * @file IPMParameters.h
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 04, 2018
 */

#ifndef LANE_LINE_IPMPARAMETERS_H
#define LANE_LINE_IPMPARAMETERS_H

#include <string>

#include "lane_line/common.h"

namespace tt
{


struct IPMParameters
{
 public:
    void setRPY(const float roll_val, const float pitch_val,
                const float yaw_val);
    void setPrinciplePoint(const float cx_val, const float cy_val);
    void setFocalLength(const float fx_val, const float fy_val);

    void setROIIn(const int x1, const int y1, const int x2, const int y2);
    void setROIIn(const cv::Rect &roi_in_val);

    void setROIOut(const int width, const int height);
    void setROIOut(const cv::Rect &roi_out_val);
 public:
    /**
     * @param filename YAML file name
     * @return true on success, false otherwise.
     */
    bool fromYAMLFile(const std::string &filename);

    bool fromProto(const IPMParametersProto &proto);
    bool toProto(IPMParametersProto &proto) const;        // NOLINT
 public:
    float pitch = 0;
    float yaw   = 0;
    float roll  = 0;

    float cx    = 0;
    float cy    = 0;
    float fx    = 0;
    float fy    = 0;
    cv::Rect roi_in;
    cv::Rect roi_out;
 public:
    std::string toString() const;
};

std::ostream& operator << (std::ostream& stream, const IPMParameters &param);


}  // namespace tt

#endif  // LANE_LINE_IPMPARAMETERS_H
