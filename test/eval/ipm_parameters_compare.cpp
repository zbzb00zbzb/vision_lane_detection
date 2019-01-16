/**
 * @file
 * @author Kuang Fangjun <csukuangfj at gmail dot com>
 * @date September 05, 2018
 *
 * Compare the CNN output for different IPM parameters
 *
 */
#include <glog/logging.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <string>

#include "lane_line/common.h"
#include "lane_line/config.h"
#include "lane_line/protobuf_util.h"

#include "lane_line/FileSystem.h"
#include "lane_line/IPMTransformation.h"
#include "lane_line/Preprocessing.h"

namespace
{

cv::Mat getGradientMask(const cv::Mat &ipm)
{
    cv::Mat gray;

    CHECK_EQ(ipm.channels(), 3);
    cv::cvtColor(ipm, gray, cv::COLOR_BGR2GRAY);

    gray.convertTo(gray, CV_32F);

    int lane_width  = 15;

    int nr = gray.rows;
    int nc = gray.cols;

    cv::Mat grad = cv::Mat::zeros(gray.size(), CV_8UC1);

    for (int r = 0; r < nr; r++)
    {
        const auto *p = gray.ptr<float>(r);
        auto *pgrad = grad.ptr<uchar>(r);
        for (int c = lane_width; c < nc - lane_width; c++)
        {
            if (p[c] < 50) continue;
            auto diff = 2*p[c] - p[c - lane_width] - p[c + lane_width];
            if (diff > 50)
            {
                pgrad[c] = 1;
            }
        }
    }

    return grad;
}

}  // namespace

int main(int, char *argv[])
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;

    std::string dir = TEST_DATA_DIR;

    cv::VideoWriter writer;
    auto video_file = dir + "/eval/2018-07-12-16-55-21-a.mp4";
    int frame_rate = 5;
    writer.open(video_file, CV_FOURCC('X', '2', '6', '4'),
                frame_rate, cv::Size(960+448*2, 448*2));

    auto config_file = dir + "/eval/2018-07-12-16-55-21-a.txt";

    tt::LaneLineDetectorProto param;
    tt::protobuf::readProtoFromTextFile(config_file, param);
    LOG(INFO) << param.DebugString();

    auto ipm_tf = tt::IPMTransformation::create(param.ipm_param());
    auto cnn = tt::Preprocessing::create(param.preprocessing_param());

    std::string image_dir = "/home/fangjun/Desktop/images/done/2018-07-12-16-55-21";    // NOLINT
    auto list_of_images = tt::FileSystem::getListOfFiles(image_dir, {".jpg"});
    LOG(INFO) << "there are " << list_of_images.size() << " images";
    cv::Mat image;
    // 188, 1600, max 1888
    int num_images = static_cast<int>(list_of_images.size());
    for (int i = 188; i < num_images-20; i++)
    {
        std::string filename = list_of_images[i];
        LOG(INFO) << "processing image "
                  << i << "/" << num_images << "\n"
                  << filename;
        LOG(INFO) << ipm_tf->param().DebugString();
        auto image = cv::imread(filename, cv::IMREAD_COLOR);
        cv::resize(image, image, cv::Size(960, 600));

        auto ipm = ipm_tf->computeIPMImage(image);
        cnn->doPreprocessing(ipm);

        auto seg = cnn->getResult();
        auto grad = getGradientMask(ipm);

        cv::Mat grad_seg = seg.mul(grad);

        cv::cvtColor(seg, seg, cv::COLOR_GRAY2BGR);
        cv::cvtColor(grad, grad, cv::COLOR_GRAY2BGR);
        grad *= 255;
        cv::cvtColor(grad_seg, grad_seg, cv::COLOR_GRAY2BGR);

        int num_rows = cv::max(image.rows, 2*ipm.rows);
        int num_cols = image.cols + ipm.cols*2;

        cv::Mat all = cv::Mat::zeros(num_rows, num_cols, CV_8UC3);
        LOG(INFO) << "all.rows: " << all.rows;
        LOG(INFO) << "all.cols: " << all.cols;

        image.copyTo(all(cv::Rect(0, 0, image.cols, image.rows)));
        ipm.copyTo(all(cv::Rect(image.cols, 0, ipm.cols, ipm.rows)));
        seg.copyTo(all(cv::Rect(image.cols, ipm.rows, seg.cols, seg.rows)));
        grad.copyTo(all(cv::Rect(image.cols+ipm.cols, 0, grad.cols, grad.rows)));       // NOLINT
        grad_seg.copyTo(all(cv::Rect(image.cols+ipm.cols, grad.rows, grad_seg.cols, grad_seg.rows)));   // NOLINT

        writer << all;

//        cv::imshow("ipm", ipm);
//        cv::imshow("seg", seg);
//        cv::imshow("image", image);
        cv::imshow("all", all);
        int key = cv::waitKey(1);
        if (key == 'q')
        {
            break;
        }
        else if (key == 'd')    // NOLINT
        {
            i -= 2;
            if (i < -1) i = -1;
        }
    }

    return 0;
}
