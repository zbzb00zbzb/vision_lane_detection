/**
 * @file Drawing.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date May 30, 2018
 */

#include <vector>
#include <iostream>
#include "lane_line/Drawing.h"
#include "lane_line/Transformation.h"

namespace tt
{

cv::Scalar Drawing::red() { return cv::Scalar(0, 0, 255);}
cv::Scalar Drawing::green() { return cv::Scalar(0, 255, 0);}
cv::Scalar Drawing::blue() { return cv::Scalar(255, 0, 0);}
cv::Scalar Drawing::black() { return cv::Scalar(0, 0, 0);}
cv::Scalar Drawing::white() { return cv::Scalar(255, 255, 255);}
cv::Scalar Drawing::yellow() { return cv::Scalar(0, 255, 255);}
cv::Scalar Drawing::cyan() { return cv::Scalar(255, 255, 0);}
cv::Scalar Drawing::magenta() { return cv::Scalar(255, 0, 255);}
cv::Scalar Drawing::purple() { return cv::Scalar(255, 48, 155);}
cv::Scalar Drawing::greenYellow() { return cv::Scalar(47, 255, 173);}
cv::Scalar Drawing::gray() { return cv::Scalar(128, 128, 128);}

cv::Mat Drawing::line(const cv::Mat &image,
                      const cv::Point2f &_p1,
                      const cv::Point2f &_p2,
                      const cv::Scalar &color /* = red() */,
                      const int line_width /* = 2 */,
                      bool clip_line /* = false */)
{
    cv::Mat result = image.clone();

    cv::Point p1 = _p1;
    cv::Point p2 = _p2;
    if (clip_line)
    {
        int start_x = 0;
        int start_y = cvRound(image.rows/3.0f);
        cv::clipLine(cv::Rect(start_x, start_y, 960, 600), p1, p2);
    }
    cv::line(result, p1, p2, color, line_width);
    return result;
}

void Drawing::lineInPlace(cv::Mat &image, // NOLINT
                          const cv::Point2f &_p1,
                          const cv::Point2f &_p2,
                          const cv::Scalar &color /* = red */,
                          const int line_width /* = 2 */,
                          bool clip_line /* = false */)
{
    cv::Point p1 = _p1;
    cv::Point p2 = _p2;
    if (p1.x < 0 || p1.x >= image.cols || p1.y < 0 || p1.y >= image.rows || p2.x < 0 || p2.x >= image.cols || p2.y < 0 || p2.y >= image.rows)
    {
        return;
    }
    if (clip_line)
    {
        int start_x = 0;
        int start_y = cvRound(image.rows/3.0f);
        cv::clipLine(cv::Rect(start_x, start_y, 1920, 1200), p1, p2);
    }
    cv::line(image, p1, p2, color, line_width);
}

cv::Mat Drawing::line(const cv::Mat &image,
                      const Line &input_line,
                      const cv::Scalar &color /* = red */,
                      const int line_width /* = 2 */,
                      bool clip_line /* = false */)
{
    std::vector<Point> points;
    input_line.computeStartEndPoints(image, points);
    if (points[0].x() < 0 || points[0].x() >= image.cols || points[0].y() < 0 || points[0].y() >= image.rows || points[1].x() < 0 || points[1].x() >= image.cols || points[1].y() < 0 || points[1].y() >= image.rows)
    {
        return line(image, cv::Point2f(0,0), cv::Point2f(0,0),
                color, 1, false);;
    }
    return line(image, cv::Point2f(points[0]), cv::Point2f(points[1]),
                color, line_width, clip_line);
}

void Drawing::lineInPlace(cv::Mat &image,  // NOLINT
                          const Line &input_line,
                          const cv::Scalar &color /* = red */,
                          const int line_width /* = 2 */,
                          bool clip_line /* = false */)
{
    std::vector<Point> points;
    input_line.computeStartEndPoints(image, points);
    // std::cout << "visualization step - point0:" << points[0].x() << "-" << points[0].y() << std::endl;
    // std::cout << "visualization step - point1:" << points[1].x() << "-" << points[1].y() << std::endl;
    lineInPlace(image, cv::Point2f(points[0]),
                cv::Point2f(points[1]), color, line_width, clip_line);
}

void Drawing::lineInPlace(cv::Mat &ipm_image,  // NOLINT
                          cv::Mat &raw_image,  // NOLINT
                          const cv::Matx33f &ipm_to_raw_tf_,    // NOLINT
                          const Line &input_line,
                          const cv::Scalar &color /*= red()*/,
                          const int line_width /* = 2 */,
                          bool clip_line /* = false */)
{
    std::vector<Point> points;
    input_line.computeStartEndPoints(ipm_image, points);
    // std::cout << "visualization step - point0:" << points[0].x() << "-" << points[0].y() << std::endl;
    // std::cout << "visualization step - point1:" << points[1].x() << "-" << points[1].y() << std::endl;
    if ((points[0].x() == -1 && points[0].y() == -1) || (points[1].x() == -1 && points[1].y() == -1))
    {
        return;
    }
    lineInPlace(ipm_image, cv::Point2f(points[0]),
                cv::Point2f(points[1]), color, line_width);

    std::vector<cv::Point2f> cv_points {cv::Point2f(points[0]),
                                        cv::Point2f(points[1])};

    cv_points = Transformation::transformPoints(ipm_to_raw_tf_, cv_points);
    tt::Line line;
    line.initWithPoints(tt::Point(cv_points[0]), tt::Point(cv_points[1]));
    line.computeStartEndPoints(raw_image, points);
    lineInPlace(raw_image, cv::Point2f(points[0]),
                cv::Point2f(points[1]), color, line_width, clip_line);
}

cv::Mat Drawing::circle(const cv::Mat &image,
                        const cv::Point2f &center,
                        const int radius,
                        const cv::Scalar &color /* = red */,
                        const int line_width /* = 2 */)
{
    cv::Mat result = image.clone();
    cv::circle(result, center, radius, color, line_width);
    return result;
}

void Drawing::circleInPlace(cv::Mat &image,  // NOLINT
                            const cv::Point2f &center,
                            const int radius,
                            const cv::Scalar &color /* = red */,
                            const int line_width /* = 2 */)
{
    cv::circle(image, center, radius, color, line_width);
}

cv::Mat Drawing::listOfPoints(const cv::Mat &image,
                              const std::vector<Point> &points,
                              const int radius,
                              const cv::Scalar &color /* = red */,
                              const int line_width /* = 2 */)
{
    auto result = image.clone();

    for (const auto &p : points)
    {
        circleInPlace(result, p, radius, color, line_width);
    }

    return result;
}

void Drawing::listOfPointsInPlace(cv::Mat &image,  // NOLINT
                                  const std::vector<Point> &points,
                                  const int radius,
                                  const cv::Scalar &color /* = red */,
                                  const int line_width /* = 2 */)
{
    for (const auto &p : points)
    {
        circleInPlace(image, p, radius, color, line_width);
    }
}

void Drawing::polyLineInPlace(cv::Mat &image,   // NOLINT
                              const std::vector<Point> &points,
                              const cv::Scalar &color /* = red */,
                              const int line_width /* = 2 */)
{
    CHECK_GE(points.size(), 2);
    int n = static_cast<int>(points.size());

    for (int i = 0; i < n - 1; i++)
    {
        lineInPlace(image, points[i], points[i+1], color, line_width);
    }
}

cv::Mat Drawing::polyLine(const cv::Mat &image,
                          const std::vector<Point> &points,
                          const cv::Scalar &color /* = red */,
                          const int line_width /* = 2 */)
{
    cv::Mat res = image.clone();

    polyLineInPlace(res, points, color, line_width);

    return res;
}

}  // end namespace tt
