/**
 * @file test_cnn_caffe_main.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date August 10, 2018
 *
 * Test the CNN interface.
 */

#include <vector>

#include "cnn/CnnInterface.h"
#include "lane_line/common.h"
#include "lane_line/config.h"

#include "lane_line/FileSystem.h"

int main(int, char *argv[])
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;

    auto cnn = tt::CnnInterface::create("caffe");
    cnn->init(CNN_PROTO_FILENAME, CNN_TRAINED_FILENAME,
              (CNN_USE_GPU ? 0 : -1));

    std::vector<cv::Mat> output;
    int run_num = 10;
    cv::TickMeter total;
    for (int i = 0; i < run_num; i++)
    {
        cv::TickMeter tm;
        std::ostringstream ss;

        // 2.jpg: general ipm
        // 3.jpg: gantry ipm
        ss << TEST_DATA_DIR << "/" << ((i%2) + 2) << ".jpg";
        LOG(INFO) << "read " << ss.str();
        auto image = cv::imread(ss.str(), cv::IMREAD_COLOR);
        CHECK(!image.empty());

        tm.start();
        total.start();
        cnn->forward(image, &output);
        tm.stop();
        total.stop();

        LOG(INFO) << "run: " << i << " " << tm.getTimeMilli() << " ms";

        LOG(INFO) << "channels: " << output.size();
        cv::imshow("general", output[2]);
        cv::imshow("gantry", output[3]);
        cv::waitKey(0);
    }

    return 0;
}

