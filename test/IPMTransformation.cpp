/**
 * @file IPMTransformation.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 04, 2018
 */

#include <gtest/gtest.h>
#include <string>

#include "lane_line/common.h"
#include "lane_line/config.h"
#include "lane_line/FileSystem.h"
#include "lane_line/IPMTransformation.h"
#include "lane_line/protobuf_util.h"

class IPMTransformationTest : public ::testing::Test
{
 protected:
    tt::IPMTransformation tf_;
};

TEST_F(IPMTransformationTest, ipm)
{
    cv::destroyAllWindows();

    std::string image_filename;
    image_filename = std::string(TEST_DATA_DIR) + "/" + "1.png";

    if (!tt::FileSystem::fileExists(image_filename))
    {
        LOG(WARNING) << "File " << image_filename << " does not exist!";
        return;
    }

    std::string ipm_filename;
    ipm_filename = std::string(TEST_DATA_DIR) + "/" + "1-ipm.txt";
    if (!tt::FileSystem::fileExists(ipm_filename))
    {
        LOG(WARNING) << "File " << ipm_filename << " does not exist!";
        return;
    }

    tt::IPMParameters param;
    auto ret = param.fromYAMLFile(ipm_filename);
    EXPECT_TRUE(ret);

    tf_.init(param);

    auto raw = cv::imread(image_filename, cv::IMREAD_COLOR);
    EXPECT_TRUE(!raw.empty());

    auto ipm = tf_.computeIPMImage(raw);
    cv::imshow("raw", raw);
    cv::imshow("ipm", ipm);
    cv::waitKey(0);
}


TEST_F(IPMTransformationTest, precomputed_H)
{
    std::string dir = TEST_DATA_DIR;
    auto config_filename = dir + "/11.txt";

    if (!tt::FileSystem::fileExists(config_filename))
    {
        LOG(ERROR) << "File " << config_filename
                   << " does not exist!\nSkip it.";
        return;
    }

    tt::IPMParametersProto proto;
    tt::protobuf::readProtoFromTextFile(config_filename, proto);

    LOG(INFO) << proto.DebugString();
    tf_.initWithPrecomputedH(proto);

    auto image_filename = dir + "/11.png";
    auto image = cv::imread(image_filename, cv::IMREAD_COLOR);
    cv::flip(image, image, -1);

    auto ipm = tf_.computeIPMImage(image);

    auto inv_ipm = tf_.computeRawImage(ipm, image.size());

    cv::namedWindow("image", cv::WINDOW_NORMAL);
    cv::namedWindow("ipm", cv::WINDOW_NORMAL);
    cv::namedWindow("inv_ipm", cv::WINDOW_NORMAL);

    cv::imshow("image", image);
    cv::imshow("ipm", ipm);
    cv::imshow("inv_ipm", inv_ipm);

    cv::waitKey(0);

    LOG(INFO) << proto.DebugString();
}

TEST_F(IPMTransformationTest, precomputed_H_2)
{
    std::string dir = TEST_DATA_DIR;
    auto config_filename = dir + "/0054.txt";

    if (!tt::FileSystem::fileExists(config_filename))
    {
        LOG(ERROR) << "File " << config_filename
                   << " does not exist!\nSkip it.";
        return;
    }

    tt::IPMParametersProto proto;
    tt::protobuf::readProtoFromTextFile(config_filename, proto);

    LOG(INFO) << proto.DebugString();
    tf_.initWithPrecomputedH(proto);

    auto image_filename = dir + "/left-0054.png";
    auto image = cv::imread(image_filename, cv::IMREAD_COLOR);

    auto ipm = tf_.computeIPMImage(image);

    cv::namedWindow("image", cv::WINDOW_NORMAL);
    cv::namedWindow("ipm", cv::WINDOW_NORMAL);

    cv::imshow("image", image);
    cv::imshow("ipm", ipm);

    cv::waitKey(0);

    LOG(INFO) << proto.DebugString();
}
