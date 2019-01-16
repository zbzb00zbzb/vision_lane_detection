/**
 * @file visualization.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date August 13, 2018
 */

#include "lane_line/visualization.h"

#include <vector>
#include <iostream>

namespace tt
{
cv::Mat visualizeResult(std::shared_ptr<tt::LaneLineDetector> detector,
                        const cv::Mat &raw_image,
                        bool draw_left_right_boundary,
                        bool only_show_original)
{
    /*
     *  ----------------------------------------------------
     *  |Segmentation_result || Narrowing_result || Curves |
     *  ----------------------------------------------------
     *  |              original image                      |
     *  ----------------------------------------------------
     */
    cv::Mat segmentation_result = detector->preprocessing()->getResult();
    cv::Mat narrowing_result = detector->narrowing()->getResult();

    cv::Mat ipm_image = detector->getIpmImage();

    cv::Mat tmp_raw_image = raw_image.clone();
    auto& curves = detector->getKalmanFilteredCenterCurves();
    // std::cout << "visualization step - center line size:" << curves.size() << std::endl;
    for (const auto &c : curves)
    {
        const auto line = dynamic_cast<const tt::Line *>(c.get());
        CHECK_NOTNULL(line);
        // std::cout << "visualization step - center line para:" << line->getThetaInDegree() << "---" << line->getD() << std::endl;
        tt::Drawing::lineInPlace(ipm_image,
                                 tmp_raw_image,
                                 detector->ipm_tf()->getIPMToImage(),
                                 *line,
                                 tt::Drawing::green(), 3, true);
    }

    if (draw_left_right_boundary)
    {
        bool clip_line = true;
        auto& left_curves = detector->getKalmanFilteredLeftCurves();
        // std::cout << "visualization step - left line size:" << left_curves.size() << std::endl;
        for (const auto &c : left_curves)
        {
            const auto line = dynamic_cast<const tt::Line *>(c.get());
            CHECK_NOTNULL(line);
            // std::cout << "visualization step - left line para:" << line->getThetaInDegree() << "---" << line->getD() << std::endl;
            tt::Drawing::lineInPlace(ipm_image,
                                     tmp_raw_image,
                                     detector->ipm_tf()->getIPMToImage(),
                                     *line,
                                     tt::Drawing::red(), 3, clip_line);
        }

        auto& right_curves = detector->getKalmanFilteredRightCurves();
        // std::cout << "visualization step - right line size:" << right_curves.size() << std::endl;
        for (const auto &c : right_curves)
        {
            const auto line = dynamic_cast<const tt::Line *>(c.get());
            CHECK_NOTNULL(line);
            // std::cout << "visualization step - right line para:" << line->getThetaInDegree() << "---" << line->getD() << std::endl;
            tt::Drawing::lineInPlace(ipm_image,
                                     tmp_raw_image,
                                     detector->ipm_tf()->getIPMToImage(),
                                     *line,
                                     tt::Drawing::blue(), 3, clip_line);
        }
    }
    if (only_show_original)
    {
        cv::resize(tmp_raw_image, tmp_raw_image, cv::Size(960, 600));
        return tmp_raw_image;
    }

    cv::Mat tmp;
    int cols = ipm_image.cols * 3;
    cv::resize(tmp_raw_image, tmp, cv::Size(cols,
                                            static_cast<int>(1.0f*raw_image.rows*raw_image.cols/cols)));  // NOLINT

    int rows = segmentation_result.rows + tmp.rows;

    cv::Mat res = cv::Mat::zeros(rows, cols, CV_8UC3);
    tmp.copyTo(res(cv::Rect(0, ipm_image.rows, tmp.cols, tmp.rows)));

    cv::cvtColor(segmentation_result, tmp, cv::COLOR_GRAY2BGR);
    tmp.copyTo(res(cv::Rect(0, 0, segmentation_result.cols, segmentation_result.rows)));        // NOLINT

    std::vector<cv::Mat> mn(3);
    cv::split(narrowing_result, mn);
    mn[1].convertTo(mn[1], CV_8UC1);
    cv::cvtColor(mn[1], tmp, cv::COLOR_GRAY2BGR);
    tmp.copyTo(res(cv::Rect(segmentation_result.cols, 0, tmp.cols, tmp.rows)));

    tmp = ipm_image.clone();
    tmp.copyTo(res(cv::Rect(segmentation_result.cols + narrowing_result.cols, 0, tmp.cols, tmp.rows)));     // NOLINT

    cv::resize(res, res, cv::Size(960, 600));
    return res;
}

}  // namespace tt

