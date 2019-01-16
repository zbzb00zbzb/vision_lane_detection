/**
 * @file IPMParameters.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 04, 2018
 */


#include <yaml-cpp/yaml.h>

#include <sstream>
#include <string>

#include "lane_line/common.h"
#include "lane_line/IPMParameters.h"

namespace tt
{

void IPMParameters::setRPY(const float roll_val, const float pitch_val,
                           const float yaw_val)
{
    roll  = roll_val;
    pitch = pitch_val;
    yaw   = yaw_val;
}

void IPMParameters::setPrinciplePoint(const float cx_val, const float cy_val)
{
    cx = cx_val;
    cy = cy_val;
}

void IPMParameters::setFocalLength(const float fx_val, const float fy_val)
{
    fx = fx_val;
    fy = fy_val;
}

void IPMParameters::setROIIn(const int x1, const int y1,
                             const int x2, const int y2)
{
    CHECK_GT(x1, 0) << "Invalid x1";
    CHECK_GT(y1, 0) << "Invalid y1";

    CHECK_GT(x2, x1) << "Invalid x2";
    CHECK_GT(y2, y1) << "Invalid y2";

    roi_in.x = x1;
    roi_in.y = y1;

    roi_in.width = x2 - x1;
    roi_in.height = y2 - y1;
}

void IPMParameters::setROIIn(const cv::Rect &roi_in_val)
{
    roi_in = roi_in_val;
}

void IPMParameters::setROIOut(const int width, const int height)
{
    roi_out.width = width;
    roi_out.height = height;
}

void IPMParameters::setROIOut(const cv::Rect &roi_out_val)
{
    roi_out = roi_out_val;
}

std::string IPMParameters::toString() const
{
    std::stringstream ss;
    ss << "#==========IPM Parameters==========\n";
#define MYSTRING(x) \
    ss << #x << ": " << (x) << "\n"

    MYSTRING(pitch);
    MYSTRING(yaw);
    MYSTRING(roll);

    MYSTRING(cx);
    MYSTRING(cy);

    MYSTRING(fx);
    MYSTRING(fy);

#undef MYSTRING

    ss << "x1: " << roi_in.x << "\n";
    ss << "y1: " << roi_in.y << "\n";

    ss << "x2: " << (roi_in.x + roi_in.width) << "\n";
    ss << "y2: " << (roi_in.y + roi_in.height) << "\n";

    ss << "width: " << roi_out.width << "\n";
    ss << "height: " << roi_out.height << "\n";

    ss << "#----------------------------------\n";

    return ss.str();
}

bool IPMParameters::fromYAMLFile(const std::string &filename)
{
    YAML::Node config = YAML::LoadFile(filename);

#define MYREAD(p, type) \
    type p ## _ ## val = 0; \
    if (config[#p]) p ## _ ## val = config[#p].as<type>(); \
    else return false;  // NOLINT

    MYREAD(roll, float);
    MYREAD(pitch, float);
    MYREAD(yaw, float);

    MYREAD(cx, float);
    MYREAD(cy, float);
    MYREAD(fx, float);
    MYREAD(fy, float);

    MYREAD(x1, int);
    MYREAD(y1, int);

    MYREAD(x2, int);
    MYREAD(y2, int);

    MYREAD(width, int);
    MYREAD(height, int);

#undef MYREAD

    roll  = roll_val;
    pitch = pitch_val;
    yaw   = yaw_val;

    cx    = cx_val;
    cy    = cy_val;
    fx    = fx_val;
    fy    = fy_val;

    roi_in.x = x1_val;
    roi_in.y = y1_val;
    roi_in.width = x2_val - x1_val;
    roi_in.height = y2_val - y1_val;

    roi_out.width = width_val;
    roi_out.height = height_val;

    CHECK_GT(x1_val, 0) << "invalid x1";
    CHECK_GT(y1_val, 0) << "invalid y1";

    CHECK_GT(x2_val, x1_val) << "invalid x2";
    CHECK_GT(y2_val, y1_val) << "invalid y2";

    CHECK_GT(cx, 0) << "invalid cx";
    CHECK_GT(cy, 0) << "invalid cy";

    CHECK_GT(fx, 0) << "invalid fx";
    CHECK_GT(fy, 0) << "invalid fy";

    CHECK_GT(roi_out.width, 0) << "invalid width";
    CHECK_GT(roi_out.height, 0) << "invalid height";

    return true;
}

bool IPMParameters::fromProto(const IPMParametersProto &proto)
{
    roll  = proto.roll();
    pitch = proto.pitch();
    yaw   = proto.yaw();

    CHECK_GT(proto.cx(), 0) << "invalid cx";
    CHECK_GT(proto.cy(), 0) << "invalid cy";
    CHECK_GT(proto.fx(), 0) << "invalid fx";
    CHECK_GT(proto.fy(), 0) << "invalid fy";

    cx = proto.cx();
    cy = proto.cy();
    fx = proto.fx();
    fy = proto.fy();

    setROIIn(proto.x1(), proto.y1(), proto.x2(), proto.y2());
    setROIOut(proto.width(), proto.height());

    return false;
}

bool IPMParameters::toProto(IPMParametersProto &proto) const  // NOLINT
{
    proto.set_roll(roll);
    proto.set_pitch(pitch);
    proto.set_yaw(yaw);

    proto.set_cx(cx);
    proto.set_cy(cy);
    proto.set_fx(fx);
    proto.set_fy(fy);

    proto.set_x1(roi_in.x);
    proto.set_y1(roi_in.y);
    proto.set_x2(roi_in.br().x);
    proto.set_y2(roi_in.br().y);

    proto.set_width(roi_out.width);
    proto.set_height(roi_out.height);

    return true;
}


std::ostream& operator << (std::ostream& stream, const IPMParameters &param)
{
    stream << param.toString() << "\n";
    return stream;
}

}  // namespace tt

