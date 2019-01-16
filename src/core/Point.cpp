/**
 * @file Point.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date May 30, 2018
 */

#include <string>

#include "lane_line/Point.h"

namespace tt
{

Point::Point()
    : x_(-1),
      y_(-1),
      left_x_(-1),
      right_x_(-1)
{
}

Point::Point(const float x, const float y)
    : x_(x),
      y_(y),
      left_x_(-1),
      right_x_(-1)
{}

Point::Point(const cv::Point2f &point)
    : x_(point.x),
      y_(point.y),
      left_x_(-1),
      right_x_(-1)
{
}

std::string Point::toString() const
{
    std::stringstream ss;

    ss << "========Point========\n";
    ss << "  x:       " << x_ << "\n";
    ss << "  y:       " << y_ << "\n";
    ss << "  left_x:  " << left_x_ << "\n";
    ss << "  right_x: " << right_x_ << "\n";

    ss << "\n";

    return ss.str();
}


std::ostream& operator << (std::ostream &stream, const Point& p)
{
    stream << p.toString();
    return stream;
}

}  // end namespace tt
