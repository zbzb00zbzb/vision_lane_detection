/**
 * @file general_main.cpp
 * @author Kuang Fangjun <csukuangfj at gmail dot com>
 * @date August 01, 2018
 *
 * @brief entry for general lane marking
 *
 */

#include <glog/logging.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>

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

void my_test(const std::shared_ptr<tt::LaneLineDetector> detector,
             const cv::Mat& image)
{
    cv::Mat gray;

    auto ipm = detector->ipm_tf()->computeIPMImage(image);

    cv::cvtColor(ipm, gray, cv::COLOR_BGR2GRAY);

    gray.convertTo(gray, CV_32F);

    int lane_width  = 10;

    int nr = gray.rows;
    int nc = gray.cols;

    cv::Mat grad = cv::Mat::zeros(gray.size(), CV_8UC1);

    for (int r = 0; r < nr; r++)
    {
        const auto *p = gray.ptr<float>(r);
        auto *pgrad = grad.ptr<uchar>(r);
        for (int c = lane_width; c < nc - lane_width; c++)
        {
            if (p[c] < 30) continue;
            auto diff = 2*p[c] - p[c - lane_width] - p[c + lane_width];
            if (diff > 50)
            {
                pgrad[c] = 255;
            }
        }
    }

    gray.convertTo(gray, CV_8U);
    cv::imshow("ipm", ipm);
    cv::imshow("gray", gray);
    cv::imshow("grad", grad);
}

static int process(std::shared_ptr<tt::LaneLineDetector> detector,
                    const cv::Mat &image, const std::string &filename,
                    cv::VideoWriter *writer);

int main(int, char *argv[])
{
    FLAGS_alsologtostderr = 1;
    FLAGS_colorlogtostderr = 1;
    google::InitGoogleLogging(argv[0]);

    bool save_video_to_file = false;

    cv::VideoWriter writer;
    if (save_video_to_file)
    {
        std::string f2 = "/shared/2018-07-21-16-55-21-without-gradient-mask.mp4";       // NOLINT
        int frame_rate = 5;
        writer.open(f2, CV_FOURCC('X', '2', '6', '4'),
                    frame_rate, cv::Size(960, 600));
        if (!writer.isOpened())
        {
            LOG(FATAL) << "cannot open " << f2;
        }
    }

    /*
     * ==============================
     *   Load config
     * ------------------------------
     */
    std::string config_filename = CONFIG_PROTO_FILENAME;

    tt::LaneLineDetectorProto param;
    tt::loadConfigFromFile(config_filename, &param);

    tt::checkConfig(param);

    std::string filename = std::string("./current-config-") +
                           tt::DateTime::toString()+ ".txt";
    tt::saveConfigToFile(filename, param);
    tt::saveConfigToFile("./latest-config.txt", param);

    auto detector = tt::LaneLineDetector::create(param);
    CHECK(detector);

    std::string image_dir;


    std::string dir = "/home/fangjun/Desktop/images/done/2018-07-12-16-55-21";
    auto list_of_images = tt::FileSystem::getListOfFiles(dir, {".jpg"});
    cv::Mat image;
    // 188, 1600, max 1888
    int num_images = static_cast<int>(list_of_images.size());
    for (int i = 188; i < num_images-20; i++)
    {
        LOG(INFO) << "Process " << i << "/" << list_of_images.size()
                  << " "
                  << list_of_images[i] << "\n";
        image = cv::imread(list_of_images[i], cv::IMREAD_COLOR);
        if (image.empty())
        {
            LOG(FATAL) << "Failed to read " << filename;
        }
        cv::resize(image, image, cv::Size(960, 600));
        my_test(detector, image);

        int key = process(detector, image, list_of_images[i],
                save_video_to_file ? &writer : NULL);
        if (key == 'd')
        {
            i -= 2;
            if (i < -1) i = -1;
        }
    }

    if (save_video_to_file)
    {
        writer.release();
    }
}

static std::string getName(const std::string &filename)
{
    auto p1 = filename.rfind("/");
    auto p2 = filename.rfind(".");
    return filename.substr(p1+1, p2-p1-1)+".png";
}

static cv::Mat drawIpmResult(std::shared_ptr<tt::LaneLineDetector> detector)
{
    cv::Mat image = cv::Mat::zeros(detector->getIpmImage().size(), CV_8UC3);
    auto& curves = detector->getKalmanFilteredCenterCurves();
    for (const auto &c : curves)
    {
        const auto line = dynamic_cast<const tt::Line *>(c.get());
        CHECK_NOTNULL(line);

        tt::Drawing::lineInPlace(image,
                                 *line,
                                 tt::Drawing::white(), 14, false);
    }
    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
    image = image > 100;
    return image;
}

static int process(std::shared_ptr<tt::LaneLineDetector> detector,
                    const cv::Mat &image, const std::string &filename,
                    cv::VideoWriter *writer)
{
    std::string res_dir = "/home/fangjun/Desktop/results/compare/estimated";
    auto name = res_dir + "/" + getName(filename);
    LOG(INFO) << filename;
    LOG(INFO) << name;
    cv::TickMeter tm;
    tm.start();
    auto curves = detector->detectCurves(image);
    tm.stop();
    LOG(ERROR) << "\n time: " << tm.getTimeMilli() << " ms, "
               << "\n " << 1./tm.getTimeSec() << " Hz\n";
    LOG(INFO) << "there are " << curves.size() << " curves";

    auto draw_boundary = detector->param().visualization_param().draw_left_right_boundary();    // NOLINT
    auto only_show_original = detector->param().visualization_param().only_show_original_image();     // NOLINT
    auto vis = tt::visualizeResult(detector, image, draw_boundary, only_show_original);     // NOLINT

    auto ipm_vis_result = drawIpmResult(detector);
    cv::imwrite(name, ipm_vis_result);

    cv::namedWindow("vis", cv::WINDOW_NORMAL);
    cv::imshow("vis", vis);
    cv::imshow("vis ipm", ipm_vis_result);

    if (writer)
    {
        *writer << vis;
    }

    LOG(INFO) << "final:";
    // LOG(INFO) << curves;

    LOG(INFO) << "merged:";
    auto merged = detector->getMergeLines()->getResult();

    detector->sortLines(&merged);
    // LOG(INFO) << merged;

    int key;
    key = cv::waitKey(1);
    if ('q' == key)
    {
        if (writer)
        {
            writer->release();
        }

        exit(0);
    }

    return key;
}
