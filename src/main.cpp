/**
 * @file main.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 07, 2018
 *
 * @brief Do not use it. Outdated!
 */

#include <algorithm>
#include <deque>
#include <iostream>     // NOLINT
#include <string>
#include <vector>

#include "lane_line/common.h"
#include "lane_line/config.h"

#include "lane_line/DateTime.h"
#include "lane_line/Drawing.h"
#include "lane_line/FileSystem.h"
#include "lane_line/IPMParameters.h"
#include "lane_line/LaneLineDetector.h"
#include "lane_line/Line.h"
#include "lane_line/protobuf_util.h"

#include "segnet/segnet.h"

namespace tt
{

LaneLineDetectorProto setUp();
void loadConfigFromFile(const std::string &filename,
                        LaneLineDetectorProto *param);

void saveConfigToFile(const std::string &filename,
                      const LaneLineDetectorProto &param);

void checkConfig(const LaneLineDetectorProto &param);

}  // namespace tt

static
void showResults(
        const tt::LaneLineDetector &detector,
        const cv::Mat &raw_image,
        const cv::Mat &ipm_image,
        const cv::Mat &segmentation_result,
        const cv::Mat &narrowing_result,
        const std::vector<std::shared_ptr<tt::Curve> > &detected_curves,           // NOLINT
        const std::vector<std::shared_ptr<tt::Curve> > &curves_after_merging,      // NOLINT
        const std::vector<std::shared_ptr<tt::Curve> > &curves_after_removing)     // NOLINT
{
    CHECK_EQ(segmentation_result.size(), narrowing_result.size());
    CHECK_EQ(segmentation_result.type(), CV_8U);
    CHECK_EQ(narrowing_result.type(), CV_8U);

    cv::Mat tmp_raw_image = raw_image.clone(), tmp_ipm_image = ipm_image.clone();   // NOLINT
    for (const auto &c : detected_curves)
    {
        const auto line = dynamic_cast<const tt::Line *>(c.get());
        CHECK_NOTNULL(line);

        tt::Drawing::lineInPlace(tmp_ipm_image,
                                 tmp_raw_image,
                                 detector.ipm_tf()->getIPMToImage(),
                                 *line,
                                 tt::Drawing::green(), 2);
    }


    cv::Mat tmp;

    int cols = segmentation_result.cols * 3;
    cv::resize(raw_image, tmp, cv::Size(cols,
                                        static_cast<int>(1.0f*raw_image.rows*raw_image.cols/cols)));  // NOLINT

    int rows = segmentation_result.rows + tmp.rows;

    cv::Mat res = cv::Mat::zeros(rows, cols, CV_8UC3);
    tmp.copyTo(res(cv::Rect(0, segmentation_result.rows, tmp.cols, tmp.rows)));

    cv::cvtColor(segmentation_result, tmp, cv::COLOR_GRAY2BGR);
    tmp.copyTo(res(cv::Rect(0, 0, segmentation_result.cols, segmentation_result.rows)));        // NOLINT

    cv::cvtColor(narrowing_result, tmp, cv::COLOR_GRAY2BGR);
    tmp.copyTo(res(cv::Rect(segmentation_result.cols, 0, tmp.cols, tmp.rows)));

    tmp = tmp_ipm_image.clone();
    tmp.copyTo(res(cv::Rect(segmentation_result.cols + narrowing_result.cols, 0, tmp.cols, tmp.rows)));     // NOLINT

    cv::resize(tmp_raw_image, tmp, cv::Size(cols,
                                        static_cast<int>(1.0f*raw_image.rows*raw_image.cols/cols)));  // NOLINT
    tmp.copyTo(res(cv::Rect(0, segmentation_result.rows, tmp.cols, tmp.rows)));     // NOLINT

    cv::resize(res, res, cv::Size(900, 600));
    cv::namedWindow("res", cv::WINDOW_NORMAL);
    cv::imshow("res", res);
    int q = cv::waitKey(1);
    if (q == 'q')
    {
        exit(0);
    }
}


/*
1. ipm
2. cnn
3. narrowing
4. points extraction
5. detect curves
6. merge curves
7. remove invalid curves
8. fill in missing curves
9. save the final detected curves
 */
static
void process(tt::LaneLineDetector &detector,        // NOLINT
             const cv::Mat &image)
{
    auto ipm_image = detector.ipm_tf()->computeIPMImage(image);

    detector.preprocessing()->doPreprocessing(ipm_image);


    auto preprocessing_result = detector.preprocessing()->getResult();

    auto raw_image_result = detector.ipm_tf()->computeRawImage(preprocessing_result, image.size());     // NOLINT
    cv::imshow("ipm result", ipm_image);
    cv::imshow("raw result", raw_image_result);

    detector.narrowing()->doNarrowing(preprocessing_result);
    auto narrowing_result     = detector.narrowing()->getResult();

    detector.points_extraction()->doExtraction(narrowing_result);

    auto points = detector.points_extraction()->getPoints();
    auto detected_curves = detector.curve_fitting()->fitCurves(points);

    // auto detected_curves = detector.detectCurves(image);

    auto merged_curves = detector.mergeCurves(detected_curves);
    auto removed_curves = detector.removeInvalidCurves(merged_curves);

    auto missed_curves = detector.fillInMissingCurves(removed_curves);


    detector.queue().push_back(missed_curves);
    if (detector.queue().size() > 3)  // TODO (fangjun): move 3 to protobuf
    {
        detector.queue().pop_front();
    }

    std::cout << "detected: \n";
    std::cout << detected_curves << "\n";
    std::cout << "end-----\n";


    auto ipm_image_detected = detector.ipm_tf()->computeIPMImage(image);
    auto ipm_image_merged= ipm_image_detected.clone();
    auto ipm_image_removed_invalid = ipm_image_detected.clone();
    auto ipm_image_missed = ipm_image_detected.clone();

    showResults(detector, image, ipm_image,
                detector.preprocessing()->getResult(),
                detector.narrowing()->getResult(),
                detected_curves,
                merged_curves,
                removed_curves);

#if 0
    for (const auto &c : detected_curves)
    {
        const auto line = dynamic_cast<const tt::Line *>(c.get());
        CHECK_NOTNULL(line);

        tt::Drawing::lineInPlace(ipm_image_detected, *line,
                                 tt::Drawing::green(), 2);
    }

    for (const auto &c : merged_curves)
    {
        const auto line = dynamic_cast<const tt::Line *>(c.get());
        CHECK_NOTNULL(line);

        tt::Drawing::lineInPlace(ipm_image_merged, *line,
                                 tt::Drawing::red(), 2);
    }

    for (const auto &c : removed_curves)
    {
        const auto line = dynamic_cast<const tt::Line *>(c.get());
        CHECK_NOTNULL(line);

        tt::Drawing::lineInPlace(ipm_image_removed_invalid, *line,
                                 tt::Drawing::red(), 2);
    }

    for (const auto &c : missed_curves)
    {
        const auto line = dynamic_cast<const tt::Line *>(c.get());
        CHECK_NOTNULL(line);

        if (line->missed_num())
        {
            tt::Drawing::lineInPlace(ipm_image_missed, *line,
                                     tt::Drawing::green(), 2);
        }
        else    // NOLINT
        {
            tt::Drawing::lineInPlace(ipm_image_missed, *line,
                                     tt::Drawing::red(), 2);
        }
    }

    cv::imshow("ipm detected", ipm_image_detected);
    cv::imshow("ipm merged", ipm_image_merged);
    cv::imshow("ipm removed", ipm_image_removed_invalid);
    cv::imshow("ipm missed", ipm_image_missed);
#endif
    cv::imshow("raw", image);
    cv::imshow("preprocessing result", preprocessing_result);   // NOLINT
    cv::imshow("narrowing result", narrowing_result);

    int key = cv::waitKey(1);
    if (key == 'q')
    {
        exit(0);
    }
}

int main(int argc, char *argv[])
{
    FLAGS_alsologtostderr = 1;
    FLAGS_colorlogtostderr = 1;
    google::InitGoogleLogging(argv[0]);

#if IS_PX2
    caffe::Caffe::SetDevice(0);
#endif
    cv::String config_filename = CONFIG_PROTO_FILENAME;

    tt::LaneLineDetectorProto param;
    param = tt::setUp();
    tt::loadConfigFromFile(config_filename, &param);
    tt::checkConfig(param);

    std::string filename = std::string("./current-config-") +
            tt::DateTime::toString()+ ".txt";
    tt::saveConfigToFile(filename, param);
    tt::saveConfigToFile("./latest-config.txt", param);

    LOG(INFO) << "\n" << tt::protobuf::protoToString(param);

    cv::String myfile = "/home/fangjun/a-1.txt";
    tt::protobuf::writeProtoToTextFile(myfile, param);

    FLAGS_alsologtostderr = 0;
    tt::LaneLineDetector detector(param);
    FLAGS_alsologtostderr = 1;

    auto detector2 = tt::LaneLineDetector::create(param);
    LOG(INFO) << detector2->type();

    std::string dir = "~/Documents/20180504/src";
#if IS_PX2
    dir = "/home/nvidia/Documents/videos/10080503/16220024/src";
#else
    dir = "/home/fangjun/Documents/videos/10080503/16220024/src";
    dir = "/home/fangjun/Documents/tusimple/lane-detection-challenge-cvpr-2017/train/clips/0531/1492727625007576150";  // NOLINT
    dir = "/home/fangjun/Documents/recording/taipingyang-0412/2018-04-12-14-11-58/src";     // NOLINT
#endif

    cv::Mat image;
    int k = 0;
    // for (int i = 935; i < 5310; i++)
    for (int i = 3000; i < 80000; i++)
    {
        auto filename = cv::format("%s/%05d.jpg", dir.c_str(), i);
        if (!tt::FileSystem::fileExists(filename))
        {
            std::cerr << "File " << filename << " does not exist! Skip it.\n";
            LOG(WARNING) << "File " << filename << " does not exist! Skip it.";
            k++;
            if (k > 10) break;
            continue;
        }
        std::cout << "Process file " << filename << "\n";
        image = cv::imread(filename, cv::IMREAD_COLOR);
        if (image.empty())
        {
            std::cerr << "Failed to read " << filename;
            LOG(FATAL) << "Failed to read " << filename;
        }
        // process(detector, image);

        cv::TickMeter tm;
        tm.start();
        auto curves = detector2->detectCurves(image);
        tm.stop();
        LOG(ERROR) << "\n time: " << tm.getTimeMilli() << " ms, "
                   << "\n " << 1./tm.getTimeSec() << " Hz\n";

        cv::imshow("preprocessing result", detector2->preprocessing()->getResult());   // NOLINT
        showResults(*detector2, image, detector2->getIpmImage(),
                    detector2->preprocessing()->getResult(),
                    detector2->narrowing()->getResult(),
                    curves,
                    curves,
                    curves);
    }

    return 0;
}

