/**
 * @file compare_ipm_raw_cnn.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date August 10, 2018
 *
 * Compare the cnn segmentation results for IPM and raw images.
 *
 * 0. disable the three functions
 * 1. enable processIPM to produce cnn results for ipm image
 * 2. disable the three functions
 * 3. enable processRaw to produce cnn results for raw image
 * 4. disable the three functions
 * 5. enable createVideo() to create a mp4 file
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
#include "lane_line/FileSystem.h"

#include "lane_line/LaneLineDetector.h"

static void processIPM(std::shared_ptr<tt::LaneLineDetector> detector,
                    const cv::Mat &image, const std::string &filename);

static void processRaw(std::shared_ptr<tt::LaneLineDetector> detector,
                       const cv::Mat &image, const std::string &filename);

static void createVideo(cv::VideoWriter *writer);

int main(int, char* argv[])
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;

#if 1
    cv::VideoWriter writer;
    std::string filename = "/shared/test.mp4";
    int frame_rate = 15;
    writer.open(filename, CV_FOURCC('X', '2', '6', '4'),
                frame_rate, cv::Size(960, 600));
    createVideo(&writer);
    writer.release();
#else

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

    std::string dir = "/home/fangjun/Desktop/images/done/2018-07-12-16-55-21";
    auto list_of_images = tt::FileSystem::getListOfFiles(dir, {".jpg"});
    cv::Mat image;
    // 188, 1600, max 1888
    for (size_t i = 188; i < list_of_images.size(); i++)
    {
        LOG(INFO) << "Process " << i << "/" << list_of_images.size()
                  << " "
                  << list_of_images[i] << "\n";
        image = cv::imread(list_of_images[i], cv::IMREAD_COLOR);
        if (image.empty())
        {
            LOG(FATAL) << "Failed to read " << filename;
        }
        cv::resize(image, image, cv::Size(960, 640));
        // processIPM(detector, image, list_of_images[i]);
        processRaw(detector, image, list_of_images[i]);
    }
#endif

    return 0;
}

static void processRaw(std::shared_ptr<tt::LaneLineDetector> detector,
                       const cv::Mat &image, const std::string &filename)
{
    int x1 = 480 - 300;
    cv::Mat sub = image(cv::Rect(x1, 0, 600, 600));
    cv::resize(sub, sub, cv::Size(448, 448));

    auto cnn = detector->preprocessing();
    cnn->doPreprocessing(sub);

    auto raw_cnn = cnn->getResult();


    auto pos = filename.rfind("/");
    auto ss = filename.substr(pos+1);
    std::string dir = "/shared";

    auto str = dir + "/sub_images/" + ss;
    cv::imwrite(str, sub);

    str = dir + "/raw_cnn/" + ss;
    cv::imwrite(str, raw_cnn);
#if 0
    cv::imshow("sub", sub);
    cv::imshow("cnn_sub", raw_cnn);
    cv::imshow("image", image);

    if ('q' == cv::waitKey(0))
    {
        exit(0);
    }
#endif
}

static void processIPM(std::shared_ptr<tt::LaneLineDetector> detector,
                    const cv::Mat &image, const std::string &filename)
{
    detector->detectCurves(image);
    auto ipm = detector->getIpmImage();
    auto ipm_cnn = detector->preprocessing()->getResult();

    cv::imshow("ipm", ipm);
    cv::imshow("ipm_cnn", ipm_cnn);
    cv::imshow("raw", image);

    auto pos = filename.rfind("/");
    auto sub = filename.substr(pos+1);

    std::string dir = "/shared";

    auto str = dir + "/images/" + sub;
    cv::imwrite(str, image);

    str = dir + "/ipm_images/" + sub;
    cv::imwrite(str, ipm);

    str = dir + "/ipm_cnn/" + sub;
    cv::imwrite(str, ipm_cnn);

    if ('q' == cv::waitKey(1))
    {
        exit(0);
    }
}

static void createVideo(cv::VideoWriter *writer)
{
    std::string dir = "/shared";

    auto images = tt::FileSystem::getListOfFiles(dir + "/images", {".jpg"});
    auto ipm_images = tt::FileSystem::getListOfFiles(dir + "/ipm_images",
                                                     {".jpg"});
    auto sub_images = tt::FileSystem::getListOfFiles(dir + "/sub_images",
                                                     {".jpg"});

    auto ipm_cnn_images = tt::FileSystem::getListOfFiles(dir + "/ipm_cnn",
                                                         {".jpg"});
    auto raw_cnn_images = tt::FileSystem::getListOfFiles(dir + "/raw_cnn",
                                                         {".jpg"});


    for (size_t i = 0; i < images.size(); i++)
    {
        cv::Mat ipm_image = cv::imread(ipm_images[i], cv::IMREAD_COLOR);
        cv::Mat ipm_cnn = cv::imread(ipm_cnn_images[i], cv::IMREAD_COLOR);

        cv::Mat sub_image = cv::imread(sub_images[i], cv::IMREAD_COLOR);
        cv::Mat raw_cnn = cv::imread(raw_cnn_images[i], cv::IMREAD_COLOR);

        cv::Mat image = cv::imread(images[i], cv::IMREAD_COLOR);

        int cols = ipm_image.rows * 4;

        float ratio = 1.0f * image.rows / image.cols;
        int rows = cols*ratio;

        cv::resize(image, image, cv::Size(cols, rows));

        cv::Mat all = cv::Mat::zeros(rows + ipm_image.rows, cols, CV_8UC3);

        ipm_image.copyTo(all(cv::Rect(0, 0, ipm_image.cols, ipm_image.rows)));
        ipm_cnn.copyTo(all(cv::Rect(ipm_image.cols, 0,
                                    ipm_image.cols, ipm_image.rows)));

        sub_image.copyTo(all(cv::Rect(sub_image.cols*2, 0,
                                      ipm_image.cols, ipm_image.rows)));

        raw_cnn.copyTo(all(cv::Rect(sub_image.cols*3, 0,
                                    ipm_image.cols, ipm_image.rows)));

        image.copyTo(all(cv::Rect(0, ipm_image.rows, image.cols, image.rows)));

        cv::resize(all, all, cv::Size(960, 600));
        *writer << all;

        cv::imshow("all", all);
        if ('q' == cv::waitKey(1))
        {
            exit(0);
        }
    }
}
