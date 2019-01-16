/**
 * @file segnet.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 05, 2018
 */

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "cnn/CnnInterface.h"

#include "lane_line/common.h"
#include "lane_line/config.h"

#include "lane_line/FileSystem.h"
#include "lane_line/IPMParameters.h"
#include "lane_line/IPMTransformation.h"

class SegnetTest : public ::testing::Test
{
 public:
    void SetUp() override
    {
        cnn_ = tt::CnnInterface::create("caffe");
        std::string model_filename = CNN_PROTO_FILENAME;
        std::string weight_filename = CNN_TRAINED_FILENAME;

        if (!tt::FileSystem::fileExists(model_filename))
        {
            LOG(WARNING) << "File " << model_filename
                         << " does not exist, skip it!";
            return;
        }

        if (!tt::FileSystem::fileExists(weight_filename))
        {
            LOG(WARNING) << "File " << weight_filename
                         << " does not exist, skip it!";
            return;
        }

        int gpu_id = 0;
        cnn_->init(model_filename, weight_filename, gpu_id);

        std::string ipm_filename = std::string(TEST_DATA_DIR) +
                                   "/" + "1-ipm.txt";
        if (!tt::FileSystem::fileExists(ipm_filename))
        {
            LOG(WARNING) << "File " << ipm_filename
                         << " does not exist, skip it!";
            return;
        }

        tt::IPMParameters param;
        auto ret = param.fromYAMLFile(ipm_filename);
        if (!ret)
        {
            LOG(WARNING) << "Failed to parse " << ipm_filename
                         << ", skip it!";
            return;
        }

        tf_.init(param);
        is_ok_ = true;
    }

 protected:
    std::shared_ptr<tt::CnnInterface> cnn_;
    tt::IPMTransformation tf_;
    bool is_ok_ = false;
};

TEST_F(SegnetTest, test)
{
    cv::destroyAllWindows();

    if (!is_ok_)
    {
        LOG(WARNING) << "initialization failed, skip it!";
        return;
    }

    std::string filename = std::string(TEST_DATA_DIR) + "/" + "1.png";
    if (!tt::FileSystem::fileExists(filename))
    {
        LOG(WARNING) << "File " << filename << " does not exist! Skip it.";
        return;
    }

    auto raw = cv::imread(filename, cv::IMREAD_COLOR);
    EXPECT_TRUE(!raw.empty());

    auto ipm = tf_.computeIPMImage(raw);

    std::vector<cv::Mat> probs;
    cnn_->forward(ipm, &probs);

    auto general_lane_line_prob = probs[2];
    auto gantry_lane_line_prob = probs[3];

    cv::imshow("raw", raw);
    cv::imshow("ipm", ipm);
    cv::imshow("general", general_lane_line_prob);
    cv::imshow("gantry", gantry_lane_line_prob);

    cv::waitKey(0);

    // width: 448, height: 448
    //  2--> general
    //  3--> gantry

    // width: 448, height 224
    // 0 background
    // 1 road
    // 2 lane
    // 3 arrow
    // 4 other
}


