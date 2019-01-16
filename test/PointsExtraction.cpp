/**
 * @file PointsExtraction.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 07, 2018
 *
 */

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "lane_line/common.h"
#include "lane_line/config.h"

#include "lane_line/FileSystem.h"
#include "lane_line/PointsExtraction.h"

#include "lane_line/Narrowing.h"
#include "lane_line/RowScanNarrowing.h"

class PointsExtractionTest : public ::testing::Test
{
 public:
    void SetUp() override
    {
        filename_ = std::string(TEST_DATA_DIR) + "/" + "1-ipm.png";
        if (!tt::FileSystem::fileExists(filename_))
        {
            LOG(WARNING) << "File " << filename_ << " does not exist!";
            return;
        }

        image_ = cv::imread(filename_, cv::IMREAD_GRAYSCALE);
        if (image_.empty())
        {
            LOG(WARNING) << "Failed to read " << filename_;
            return;
        }

        param_.set_max_x_dir_search(20);
        param_.set_max_y_dir_search(20);

        is_ok_ = true;
    }

 protected:
    bool is_ok_ = false;
    std::shared_ptr<tt::PointsExtraction> pe_;
    tt::PointsExtractionProto  param_;
    std::string filename_;
    cv::Mat image_;
};

TEST_F(PointsExtractionTest, do_points_extraction)
{
    cv::destroyAllWindows();

    if (!is_ok_)
    {
        LOG(WARNING) << "Failed in SetUp().";
        return;
    }

    auto row_scan_proto = new tt::RowScanProto();
    row_scan_proto->set_threshold(200);
    row_scan_proto->set_minimum_width(3);

    tt::NarrowingProto narrowing_proto;
    narrowing_proto.set_allocated_row_scan_param(row_scan_proto);

    auto narrowing = std::make_shared<tt::RowScanNarrowing>(narrowing_proto);
    narrowing->doNarrowing(image_);

    auto image = narrowing->getResult();

    pe_ = std::make_shared<tt::PointsExtraction>(param_);
    pe_->doExtraction(image);

    auto points = pe_->getPoints();

    cv::destroyAllWindows();

    cv::imshow("points extraction [original image]", image_);
    cv::imshow("points extraction [narrowing]", image);

    cv::Mat current = cv::Mat::zeros(image.size(), CV_8U);
    cv::Mat left = cv::Mat::zeros(image.size(), CV_8U);
    cv::Mat right = cv::Mat::zeros(image.size(), CV_8U);
    cv::Mat total = cv::Mat::zeros(image.size(), CV_8UC3);

    cv::Mat m[3] = {left, current, right};
    for (const auto &pp : points)
    {
        left = 0;
        right = 0;
        current = 0;
        total = cv::Scalar(0, 0, 0);
        for (const auto &p : pp)
        {
            current.at<uchar>(cv::Point2i(p.x(), p.y())) = 255;
            left.at<uchar>(cv::Point2i(p.getLeftX(), p.y())) = 255;
            right.at<uchar>(cv::Point2i(p.getRightX(), p.y())) = 255;
        }

        cv::merge(m, 3, total);

        cv::imshow("points extraction [points] current", current);
        cv::imshow("points extraction [points] left", left);
        cv::imshow("points extraction [points] right", right);
        cv::imshow("points extraction [points] total", total);
        cv::waitKey(0);
    }
}
