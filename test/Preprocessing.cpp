/**
 * @file Preprocessing.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 06, 2018
 */

#include <gtest/gtest.h>
#include <string>

#include "lane_line/config.h"

#include "lane_line/FileSystem.h"

#include "lane_line/IPMTransformation.h"
#include "lane_line/Preprocessing.h"
#include "lane_line/SegnetPreprocessing.h"

class PreprocessingTest : public ::testing::Test
{
 public:
    void SetUp() override
    {
        tt::IPMParametersProto ipm_param_proto;

        ipm_param_proto.set_pitch(-1.1);
        ipm_param_proto.set_yaw(0.1);
        ipm_param_proto.set_roll(-1.2);

        ipm_param_proto.set_cx(640);
        ipm_param_proto.set_cy(360);
        ipm_param_proto.set_fx(700);
        ipm_param_proto.set_fy(700);
        ipm_param_proto.set_x1(549);
        ipm_param_proto.set_y1(409);
        ipm_param_proto.set_x2(749);
        ipm_param_proto.set_y2(560);

        ipm_param_proto.set_width(448);
        ipm_param_proto.set_height(448);
        ipm_tf_ = std::make_shared<tt::IPMTransformation>(ipm_param_proto);

        preprocessing_proto_.set_model_file(CNN_PROTO_FILENAME);
        preprocessing_proto_.set_trained_file(CNN_TRAINED_FILENAME);
        preprocessing_proto_.set_use_gpu(CNN_USE_GPU);
        preprocessing_proto_.set_lane_type(
                tt::MarkingType::Normal_Lane_Marking);

        image_filename_ = std::string(TEST_DATA_DIR) + "/" + "1.png";
        if (!tt::FileSystem::fileExists(image_filename_))
        {
            LOG(WARNING) << "image " << image_filename_ << "does not exist";
            return;
        }

        image_ = cv::imread(image_filename_, cv::IMREAD_COLOR);
        if (image_.empty())
        {
            LOG(WARNING) << "Failed to read " << image_filename_;
            return;
        }

        is_ok_ = true;
    }

    void TearDown() override {}
 protected:
    std::shared_ptr<tt::IPMTransformation> ipm_tf_;
    std::shared_ptr<tt::Preprocessing> preprocessing_;
    tt::PreprocessingProto preprocessing_proto_;

    std::string image_filename_;
    cv::Mat image_;

    bool is_ok_ = false;
};

TEST_F(PreprocessingTest, segnet_test)
{
    cv::destroyAllWindows();

    if (!is_ok_)
    {
        LOG(WARNING) << "skip it";
        return;
    }

    auto ipm_image = ipm_tf_->computeIPMImage(image_);
    preprocessing_ =
            std::make_shared<tt::SegnetPreprocessing>(preprocessing_proto_);
    preprocessing_->doPreprocessing(ipm_image);

    auto result = preprocessing_->getResult();

    cv::imshow("segnet_preprocessing_image", image_);
    cv::imshow("segnet_preprocessing_result", result);
    cv::waitKey(0);
}

