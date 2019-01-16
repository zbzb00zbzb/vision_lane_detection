/**
 * @file Narrowing.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 07, 2018
 */

#include <gtest/gtest.h>

#include <memory>  // for std::shared_ptr
#include <string>
#include <vector>

#include "lane_line/config.h"

#include "lane_line/DistTransNarrowing.h"
#include "lane_line/FileSystem.h"
#include "lane_line/Narrowing.h"
#include "lane_line/RowScanNarrowing.h"


class NarrowingTest : public ::testing::Test
{
 public:
    void SetUp() override
    {
        filename_ = std::string(TEST_DATA_DIR) + "/" + "1-ipm.png";
        if (!tt::FileSystem::fileExists(filename_))
        {
            LOG(WARNING) << "File " << filename_ << "does not exist!";
            return;
        }

        image_ = cv::imread(filename_, cv::IMREAD_GRAYSCALE);
        if (image_.empty())
        {
            LOG(WARNING) << "Failed to read " << filename_;
            return;
        }
        is_ok_ = true;

        auto row_scan_proto = new tt::RowScanProto();
        row_scan_proto->set_threshold(200);
        row_scan_proto->set_minimum_width(3);
        param_.set_allocated_row_scan_param(row_scan_proto);

        auto dist_trans_proto = new tt::DistTransProto();
        dist_trans_proto->set_intensity_threshold(200);
        dist_trans_proto->set_dist_threshold(25);
        param_.set_allocated_dist_trans_param(dist_trans_proto);
    }

 protected:
    std::string filename_;
    cv::Mat image_;
    std::shared_ptr<tt::Narrowing> narrowing_;
    tt::NarrowingProto param_;
    bool is_ok_ = false;
};


TEST_F(NarrowingTest, row_scan_narrowing)
{
    cv::destroyAllWindows();

    if (!is_ok_)
    {
        LOG(WARNING) << "Failure in SetUp(). Skip it!";
        return;
    }

    narrowing_ = std::make_shared<tt::RowScanNarrowing>(param_);

    narrowing_->doNarrowing(image_);
    auto result = narrowing_->getResult();
    std::vector<cv::Mat> m(3);
    cv::split(result, m);

    m[1].convertTo(m[1], CV_8U);

    cv::destroyAllWindows();
    cv::imshow("row scan narrow [original]", image_);
    cv::imshow("row scan narrow [result]", m[1]);
    cv::waitKey(10);
}

TEST_F(NarrowingTest, dist_trans_narrowing)
{
    if (!is_ok_)
    {
        LOG(WARNING) << "Failure in SetUp(). Skip it!";
        return;
    }

    narrowing_ = std::make_shared<tt::DistTransNarrowing>(param_);

    auto row_scan_proto = new tt::RowScanProto();
    row_scan_proto->set_threshold(3);
    row_scan_proto->set_minimum_width(3);
    param_.set_allocated_row_scan_param(row_scan_proto);

    auto p = std::make_shared<tt::RowScanNarrowing>(param_);

    narrowing_->doNarrowing(image_);
    auto result = narrowing_->getResult();

    p->doNarrowing(result);
    auto result2 = p->getResult();

    cv::imshow("dist trans narrow [original]", image_);
    cv::imshow("dist trans narrow [result]", result*50);
    cv::imshow("dist trans narrow [result2]", result2);
    cv::waitKey(0);
}
