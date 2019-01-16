/**
 * @file evaluation.cpp
 * @author Kuang Fangjun <csukuangfj at gmail dot com>
 * @date August 10, 2018
 *
 * This program performs the following actions:
 *  - read a lane marking mask ground truth
 *  - extract the lane marking mask region
 *  - resize the mask to 960x600
 *  - compute the IPM of the mask
 *  - do narrowing
 *  - points extraction
 *  - curve fitting
 *  - draw detected lines with a given width
 *
 * Note that no curve merge or removal are performed since it is a ground truth mask.
 */

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>

#include <memory>
#include <string>

#include "lane_line/common.h"
#include "lane_line/config.h"

#include "lane_line/DateTime.h"
#include "lane_line/Drawing.h"
#include "lane_line/FileSystem.h"

#include "lane_line/LaneLineDetector.h"

static cv::Mat getMask(const std::string &filename)
{
    auto image = cv::imread(filename, cv::IMREAD_COLOR);

    auto *data = image.ptr<cv::Vec3b>(0);
    for (int i = 0; i < image.total(); i++)
    {
        if (data[0][0] == 0 && data[0][1] == 255 && data[0][2] == 0)
        {
            data[0][0] = 255;
            data[0][1] = 255;
            data[0][2] = 255;
        }
        else  // NOLINT
        {
            data[0][0] = 0;
            data[0][1] = 0;
            data[0][2] = 0;
        }
        data++;
    }

    // cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);

    return image;
}

int main(int, char* argv[])
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;

    /*
     * ==============================
     *   Load config
     * ------------------------------
     */
    std::string config_filename = CONFIG_PROTO_FILENAME;

    tt::LaneLineDetectorProto param;
    tt::loadConfigFromFile(config_filename, &param);
    if (!param.mutable_preprocessing_param()->has_model_file())
    {
        *(param.mutable_preprocessing_param()->mutable_model_file())
                = CNN_PROTO_FILENAME;
    }

    if (!param.mutable_preprocessing_param()->has_trained_file())
    {
        *(param.mutable_preprocessing_param()->mutable_trained_file())
                = CNN_TRAINED_FILENAME;
    }

    tt::checkConfig(param);

    /*
     * ==============================
     *   Save config
     * ------------------------------
     */
    std::string filename = std::string("./current-config-") +
                           tt::DateTime::toString()+ ".txt";
    tt::saveConfigToFile(filename, param);
    tt::saveConfigToFile("./latest-config.txt", param);

    auto detector = tt::LaneLineDetector::create(param);
    CHECK(detector);

    std::string dir = "/home/fangjun/Desktop/results/sent-2018-07-20/2018-07-12-16-55-21-afternoon-sunny/Lane marking-mask";    // NOLINT
    auto images = tt::FileSystem::getListOfFiles(dir, {".png"});
    LOG(INFO) << "There are " << images.size() << " images";

    auto mask = getMask(images[55]);

    cv::resize(mask, mask, cv::Size(960, 600), 0, 0, cv::INTER_NEAREST);

    auto ipm_mask = detector->ipm_tf()->computeIPMImage(mask);
    cv::cvtColor(ipm_mask, ipm_mask, cv::COLOR_BGR2GRAY);
    detector->narrowing()->doNarrowing(ipm_mask);
    auto narrow_mask = detector->narrowing()->getResult();

    cv::imshow("mask", mask);
    cv::imshow("ipm mask", ipm_mask);

    cv::Mat mv[3];
    cv::split(narrow_mask, mv);
    mv[1].convertTo(mv[1], CV_8U);
    cv::imshow("narrow mask", mv[1]);

    detector->points_extraction()->doExtraction(narrow_mask);
    auto points = detector->points_extraction()->getPoints();

    auto curves = detector->curve_fitting()->fitCurves(points);
    LOG(INFO) << curves;

    cv::Mat res = cv::Mat::zeros(ipm_mask.size(), CV_8UC3);

    for (const auto &c : curves)
    {
        const auto line = c->getLinePtr();
        CHECK_NOTNULL(line);

        tt::Drawing::lineInPlace(res,
                                 *line,
                                 tt::Drawing::blue(), 10);
    }


    cv::imshow("lines", res);

    cv::waitKey(0);

    return 0;
}
