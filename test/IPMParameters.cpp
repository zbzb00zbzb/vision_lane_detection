/**
 * @file IPMParameters.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 04, 2018
 */


#include <gtest/gtest.h>
#include <iostream>  // NOLINT
#include <string>  // NOLINT

#include "lane_line/common.h"
#include "lane_line/config.h"

#include "lane_line/FileSystem.h"
#include "lane_line/IPMParameters.h"

class IPMParametersTest : public ::testing::Test
{
 protected:
    tt::IPMParameters p;
};

TEST_F(IPMParametersTest, set)
{
    const float eps = 1e-6;
    float roll = 1;
    float pitch = 2;
    float yaw = 3;

    float cx = 4;
    float cy = 5;

    float fx = 6;
    float fy = 7;

    cv::Rect roi_in(8, 9, 10, 11);
    cv::Rect roi_out(12, 13, 14, 15);

    p.setRPY(roll, pitch, yaw);
    p.setPrinciplePoint(cx, cy);
    p.setFocalLength(fx, fy);
    p.setROIIn(roi_in);
    p.setROIOut(roi_out);

    EXPECT_NEAR(roll, p.roll, eps);
    EXPECT_NEAR(pitch, p.pitch, eps);
    EXPECT_NEAR(yaw, p.yaw, eps);

    EXPECT_NEAR(cx, p.cx, eps);
    EXPECT_NEAR(cy, p.cy, eps);

    EXPECT_NEAR(fx, p.fx, eps);
    EXPECT_NEAR(fy, p.fy, eps);

    EXPECT_EQ(roi_in.x, p.roi_in.x);
    EXPECT_EQ(roi_in.y, p.roi_in.y);
    EXPECT_EQ(roi_in.width, p.roi_in.width);
    EXPECT_EQ(roi_in.height, p.roi_in.height);

    EXPECT_EQ(roi_out.x, p.roi_out.x);
    EXPECT_EQ(roi_out.y, p.roi_out.y);
    EXPECT_EQ(roi_out.width, p.roi_out.width);
    EXPECT_EQ(roi_out.height, p.roi_out.height);

    std::cout << p.toString();
    std::cout << p;
}

TEST_F(IPMParametersTest, from_yaml_file)
{
    std::string filename = std::string(TEST_DATA_DIR) + "/" + "ipm.txt";
    if (!tt::FileSystem::fileExists(filename))
    {
        LOG(WARNING) << "File " << filename << " does not exist, skip it!";
        return;
    }

    auto res = p.fromYAMLFile(filename);
    LOG(INFO) << "\n" << p.toString();
    EXPECT_TRUE(res);
}
