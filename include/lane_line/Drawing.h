/**
 * @file Drawing.h
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date May 30, 2018
 */

#ifndef LANE_LINE_DRAWING_H
#define LANE_LINE_DRAWING_H

#include <vector>

#include "lane_line/common.h"
#include "lane_line/Line.h"
#include "lane_line/Point.h"

namespace tt
{

class Drawing
{
 public:
    static cv::Scalar red();
    static cv::Scalar green();
    static cv::Scalar blue();
    static cv::Scalar black();
    static cv::Scalar white();
    static cv::Scalar yellow();
    static cv::Scalar cyan();
    static cv::Scalar magenta();
    static cv::Scalar purple();
    static cv::Scalar greenYellow();
    static cv::Scalar gray();


    static cv::Mat line(const cv::Mat& image,
                        const cv::Point2f &p1,
                        const cv::Point2f &p2,
                        const cv::Scalar &color = red(),
                        const int line_width = 2,
                        bool clip_line = false);

    static void lineInPlace(cv::Mat &image, // NOLINT
                            const cv::Point2f &p1,
                            const cv::Point2f &p2,
                            const cv::Scalar &color = red(),
                            const int line_width = 2,
                            bool clip_line = false);

    static cv::Mat line(const cv::Mat &image,
                        const Line &input_line,
                        const cv::Scalar &color = red(),
                        const int line_width = 2,
                        bool clip_line = false);

    static void lineInPlace(cv::Mat &image,  // NOLINT
                            const Line &input_line,
                            const cv::Scalar &color = red(),
                            const int line_width = 2,
                            bool clip_line = false);

    static void polyLineInPlace(cv::Mat &image,     // NOLINT
                                const std::vector<Point> &points,
                                const cv::Scalar &color = red(),
                                const int line_width = 2);

    static cv::Mat polyLine(const cv::Mat &image,
                            const std::vector<Point> &points,
                            const cv::Scalar &color = red(),
                            const int line_width = 2);

    /**
     *
     * @param ipm_image
     * @param raw_image
     * @param ipm_to_raw_tf_    ipm to raw image transformation
     * @param input_line  line in the ipm image
     * @param color
     * @param line_width
     */
    static void lineInPlace(cv::Mat &ipm_image,  // NOLINT
                            cv::Mat &raw_image,  // NOLINT
                            const cv::Matx33f &ipm_to_raw_tf_,    // NOLINT
                            const Line &input_line,
                            const cv::Scalar &color = red(),
                            const int line_width = 2,
                            bool clip_line = false);

    static cv::Mat circle(const cv::Mat &image,
                          const cv::Point2f &center,
                          const int radius,
                          const cv::Scalar &color = red(),
                          const int line_width = 2);

    static void circleInPlace(cv::Mat &image,  // NOLINT
                              const cv::Point2f &center,
                              const int radius,
                              const cv::Scalar &color = red(),
                              const int line_width = 2);

    static cv::Mat listOfPoints(const cv::Mat &image,
                                const std::vector<Point> &points,
                                const int radius,
                                const cv::Scalar &color = red(),
                                const int line_width = 2);

    static void listOfPointsInPlace(cv::Mat &image,  // NOLINT
                                    const std::vector<Point> &points,
                                    const int radius,
                                    const cv::Scalar &color = red(),
                                    const int line_width = 2);
};

}  // end namespace tt


#endif  // LANE_LINE_DRAWING_H
