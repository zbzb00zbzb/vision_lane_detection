/**
 * @file Point.h
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date May 30, 2018
 */

#ifndef LANE_LINE_POINT_H
#define LANE_LINE_POINT_H

#include <string>

#include "lane_line/common.h"

namespace tt
{

/**
 * This class is similar to cv::Point2f.
 * In addition, it saves the left neighbor (left_x, y) and
 * the right neighbor (right_x, y).
 */
class Point
{
 public:
    Point();
    Point(const float x, const float y);

    explicit Point(const cv::Point2f& point);
    operator cv::Point2f() const {return cv::Point2f(x_, y_);}

 public:
    void setXY(const float x, const float y) {x_ = x; y_= y;}
    float getX() const { return x_;}
    float getY() const { return y_;}

    float& x() { return x_;}
    float& y() { return y_;}

    const float& x() const {return x_;}
    const float& y() const {return y_;}

    void setXY(const cv::Point2f &point) {x_ = point.x; y_ = point.y;}
    cv::Point2f getXY() const {return cv::Point2f(x_, y_);}

    void setLeftX(const float val) {left_x_ = val;}
    float getLeftX() const {return left_x_;}
    cv::Point2f getLeftXY() const {return cv::Point2f(left_x_, y_);}

    void setRightX(const float val) {right_x_ = val;}
    float getRightX() const {return right_x_;}
    cv::Point2f getRightXY() const {return cv::Point2f(right_x_, y_);}

 public:
    std::string toString() const;

 private:
    float x_;
    float y_;
    float left_x_;
    float right_x_;
};

std::ostream& operator << (std::ostream &stream, const Point& p);

}  // end namespace tt


#endif  // LANE_LINE_POINT_H
