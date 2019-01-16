/**
 * @file Line.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date May 30, 2018
 */

#include "lane_line/Line.h"

#include <iostream>     // NOLINT
#include <string>
#include <vector>

// #define MY_RAD_TO_DEG(x) ((x)*180.f/static_cast<float>(CV_PI))
#define MY_DEG_TO_RAD(x) ((x)/180.f*static_cast<float>(CV_PI))

namespace tt
{

Line::Line()
    : theta_in_radian_(0),
      d_(0)
{}


Line::Line(const float theta_in_radian, const float d)
{
    initWithRadian(theta_in_radian, d);
}

void Line::initWithDegree(const float theta_in_degree, const float d)
{
    CHECK_GE(theta_in_degree, 0) << "Invalid theta";

    CHECK_LT(theta_in_degree, 360) << "Invalid theta.";
    CHECK_GE(d, 0) << "Invalid d";

    theta_in_radian_ = MY_DEG_TO_RAD(theta_in_degree);
    d_ = d;
}

void Line::initWithRadian(const float theta_in_radian, const float d)
{
    CHECK_GE(theta_in_radian, 0) << "Invalid theta.";
    CHECK_LT(theta_in_radian, CV_2PI) << "Invalid theta.";
    CHECK_GE(d, 0) << "Invalid d";

    theta_in_radian_ = theta_in_radian;
    d_ = d;
}

void Line::initWithPoints(const Point &p1, const Point &p2)
{
    double diff = cv::norm(cv::Point2f(p1) - cv::Point2f(p2));
    CHECK_GT(diff, 1e-2) << "The two points are too close!\n"
                << "p1: " << p1 << "\n"
                << "p2: " << p2;

    // case 1: vertical line
    if (fabsf(p1.x() - p2.x()) < 1e-5)
    {
        theta_in_radian_ = (p1.x() > 0) ? 0 : static_cast<float>(CV_PI);
        d_ = fabsf(p1.x());
        return;
    }

    // case 2: a horizontal line
    if (fabsf(p1.y() - p2.y()) < 1e-5)
    {
        theta_in_radian_ = (p1.y() > 0) ? static_cast<float>(CV_PI)*0.5f
                                      : static_cast<float>(CV_PI)*1.5f;
        d_ = fabsf(p1.y());
        return;
    }

    float slope_theta;
    slope_theta = std::atan2(p2.y() - p1.y(), p2.x() - p1.x());  // (-pi,pi]
    slope_theta = (slope_theta < 0) ? (slope_theta+static_cast<float>(CV_PI))
                                    : slope_theta;

    theta_in_radian_ = slope_theta + static_cast<float>(CV_PI)/2;
    d_ = cosf(theta_in_radian_) * p1.x() + sinf(theta_in_radian_) * p1.y();
    if (fabsf(d_) < 1e-6)
    {
        // todo: handle theta_in_radian when the line passes through the origin
        d_ = 0;
    }

    if (d_ < 0)
    {
        theta_in_radian_ += static_cast<float>(CV_PI);
        if (theta_in_radian_ > static_cast<float>(CV_2PI))
        {
            theta_in_radian_ -= static_cast<float>(CV_2PI);
        }
        d_ = -d_;
    }

    CHECK_GE(d_, 0) << "Error in computing d!";

    CHECK_GE(theta_in_radian_, 0) << "Error in computing theta!";

    CHECK_LT(theta_in_radian_, CV_2PI) << "Error in computing d!";

    inliers_.clear();
    inliers_.push_back(p1);
    inliers_.push_back(p2);
}

void Line::initWithPoints(const std::vector<Point> &points)
{
    CHECK_GE(points.size(), 2) << "It should contain at least 2 points!";

    if (points.size() == 2)
    {
        initWithPoints(points[0], points[1]);
        return;
    }

    // use opencv to fit the line.
    std::vector<cv::Point2f> cv_points;
    for (const auto &p : points) cv_points.push_back(p);

    // ax+by+c=0
    cv::Vec4f vec;  // [b, -a, x, y]
    cv::fitLine(cv_points, vec, CV_DIST_L2, 0, 0.01, 0.01);
    float a = -vec[1];
    float b = vec[0];
    float x = vec[2];
    float y = vec[3];
    float c = -(a*x + b*y);
    float x2, y2;
    if (fabsf(b) < 1e-5)
    {
        // this is a vertical line
        x2 = x;
        y2 = y + 100;  // any y different from y is fine here, we use 100 here
    }
    else // NOLINT
    {
        x2 = x + 100;  // any x different from x is fine here, we use 100 here
        y2 = -(a*x2 + c)/b;
    }

    initWithPoints(Point(x, y), Point(x2, y2));

    inliers().clear();
    inliers() = points;
}

float Line::pointAbsDistanceToThisCurve(const Point &point) const
{
    float distance = 0;
    distance = point.getX() * cosf(theta_in_radian_) +
            point.getY() * sinf(theta_in_radian_) - d_;

    distance = (distance > 0) ? distance : -distance;

    return distance;
}

std::shared_ptr<Curve> Line::createCopy() const
{
    auto result = std::make_shared<Line>();
    result->initWithRadian(theta_in_radian_, d_);
    result->inliers() = inliers();
    result->missed_num() = missed_num();
    return result;
}

Point Line::pointProjectedOnLine(const Point &point) const
{
    float c = cosf(theta_in_radian_);
    float s = sinf(theta_in_radian_);
    if (fabsf(s) < 1e-5)
    {
        // if it is  a vertical line
        Point res;

        res.x() = (c > 0) ? d_ : -d_;
        res.y() = point.y();

        return res;
    }

    // get two points on the line
    Point p1;
    Point p2;
    p1.x() = 10;
    p1.y() = (d_ - p1.x() * c) / s;

    p2.x() = 100;
    p2.y() = (d_ - p2.x() * c) / s;

    cv::Vec2f v12(p2.x() - p1.x(), p2.y() - p1.y());

    cv::Vec2f v(point.x() - p1.x(), point.y() - p1.y());

    float num = v12.dot(v);
    float den = v12.dot(v12);
    float t = num/den;
    Point res;
    res.x() = p1.x() + t*v12[0];
    res.y() = p1.y() + t*v12[1];

    return res;
}

void Line::computeStartEndPoints(const cv::Mat &image,
                                 std::vector<Point> &points) const  // NOLINT
{
    Point start_point(-1, -1);
    Point end_point(-1, -1);
    float c = cosf(theta_in_radian_);
    float s = sinf(theta_in_radian_);
    float d = d_;

    if (fabsf(s) < 1e-2)  // a vertical line
    {
        float dist = (c > 0) ? d : -d;

        start_point.x() = dist;
        start_point.y() = 0;

        end_point.x() = dist;
        end_point.y() = image.rows - 1;
    }
    else if (fabsf(c) < 1e-2)  // a horizontal line  // NOLINT
    {
        float dist = (s > 0) ? d : -d;
        start_point.x() = 0;
        start_point.y() = dist;

        end_point.x() = image.cols - 1;
        end_point.y() = dist;
    }
    else  // NOLINT
    {
        // x * c + y * s = d
        // y = (d - x*c)/s;
        // x = (d - y*s)/c;
        int y_down, y_up;
        for (y_down = 0; y_down < image.rows; y_down++)
        {
            int x = (d - y_down*s)/c;
            if (x >= 0 && x < image.cols)
            {
                start_point.x() = x;
                start_point.y() = y_down;
                break;
            }
        }

        for (y_up = image.rows - 1; y_up > y_down; y_up--)
        {
            int x = (d - y_up*s)/c;
            if (x >= 0 && x < image.cols)
            {
                end_point.x() = x;
                end_point.y() = y_up;
                break;
            }
        }

        if (y_down == y_up)
        {
            start_point.x() = -1;
            start_point.y() = -1;
            end_point.x() = -1;
            end_point.y() = -1;
        }
    }

    points.clear();
    points.push_back(start_point);
    points.push_back(end_point);
}

bool Line::isSimilarTo(const std::shared_ptr<Curve> curve,
                       const float *thresholds,
                       const int n) const
{
    CHECK_NOTNULL(curve.get());
    CHECK_EQ(curve->type(), this->type());

    CHECK_NOTNULL(thresholds);
    CHECK_EQ(n, 2) << "It needs exactly two thresholds.";

    const auto other = dynamic_cast<const Line*>(curve.get());
    CHECK_NOTNULL(other);

    float diff_in_theta = fabsf(other->getThetaInDegree() - getThetaInDegree());

    bool res;
    res = diff_in_theta <= thresholds[0];

    float diff_in_d = fabsf(other->getD() - getD());
    res &= diff_in_d <= thresholds[1];

    return res;
}

std::string Line::toString() const
{
    std::stringstream ss;
    ss << "\n==========Line==========\n";

    ss << "  theta in degree: " << getThetaInDegree() << "\n";
    ss << "  d:               " << getD() << "\n";
    ss << "  inliers count:   " << inliers().size() << "\n";
    ss << "  confidence:      " << getConfidence() << "\n";
    ss << "  missed number:   " << missed_num() << "\n";

    ss << "------------------------\n";

    return ss.str();
}

float
Line::absDistanceToThisCurve(const std::shared_ptr<Curve> &other_curve) const
{
    // assume that the two lines are parallel
    const auto other_line = dynamic_cast<const Line *>(other_curve.get());
    CHECK_NOTNULL(other_line);

    float res = getD() - other_line->getD();

    return (res > 0) ? res : -res;
}

}  // end namespace tt

#undef MY_DEG_TO_RAD
