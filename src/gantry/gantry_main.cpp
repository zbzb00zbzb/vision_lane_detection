/**
 * @file gantry_main.cpp
 * @author Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 26, 2018
 *
 */
#include <glog/logging.h>

#include <string>

#include "lane_line/common.h"
#include "lane_line/config.h"
#include "lane_line/protobuf_util.h"

#include "lane_line/DateTime.h"
#include "lane_line/Drawing.h"
#include "lane_line/FileSystem.h"
#include "lane_line/LaneLineDetector.h"
#include "lane_line/Line.h"
#include "lane_line/visualization.h"

static void process(std::shared_ptr<tt::LaneLineDetector> detector,
                    const cv::Mat &image);

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

    std::string image_dir;

    image_dir = "/home/fangjun/Documents/recording/taipingyang-0412/2018-04-12-14-11-58/src";     // NOLINT
    image_dir = "/home/fangjun/Desktop/yj";
    int k = 0;
    cv::Mat image;
    auto list_of_images = tt::FileSystem::getListOfFiles(image_dir, {".png"});
    for (int i = 0; i < list_of_images.size(); i++)
    {
        auto filename = list_of_images[i];

        if (!tt::FileSystem::fileExists(filename))
        {
            LOG(ERROR) << "File " << filename << " does not exist! Skip it.\n";
            // if (++k > 10) break;
            continue;
        }

        LOG(INFO) << "Process file " << filename << "\n";
        image = cv::imread(filename, cv::IMREAD_COLOR);
        if (image.empty())
        {
            LOG(FATAL) << "Failed to read " << filename;
        }

        cv::flip(image, image, -1);
        process(detector, image);
    }

    return 0;
}

static void process(std::shared_ptr<tt::LaneLineDetector> detector,
                    const cv::Mat &image)
{
    cv::TickMeter tm;
    tm.start();
    auto curves = detector->detectCurves(image);

    tm.stop();
    LOG(ERROR) << "\n time: " << tm.getTimeMilli() << " ms, "
               << "\n " << 1./tm.getTimeSec() << " Hz\n";

    auto draw_boundary = detector->param().visualization_param().draw_left_right_boundary();    // NOLINT
    auto only_show_original = detector->param().visualization_param().only_show_original_image();     // NOLINT
    auto vis = tt::visualizeResult(detector, image, draw_boundary, only_show_original);         // NOLINT

    cv::namedWindow("vis", cv::WINDOW_NORMAL);
    cv::imshow("vis", vis);

    if ('q' == cv::waitKey(0))
    {
        exit(0);
    }
}

