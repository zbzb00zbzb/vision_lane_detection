/**
 * @file protobuf.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 06, 2018
 */

#include <gtest/gtest.h>
#include <string>

#include "lane_line/common.h"
#include "lane_line/protobuf_util.h"

class ProtobufTest : public ::testing::Test
{
 protected:
    tt::PreprocessingProto preprocessingProto_;
};

TEST_F(ProtobufTest, preprocessing)
{
    std::string filename = "./preprocessing.txt";
    preprocessingProto_.set_use_gpu(true);
    preprocessingProto_.set_device_id(2);
    preprocessingProto_.set_model_file("deploy.prototxt");
    preprocessingProto_.set_trained_file("after_bn_calc2.caffemodel");
    preprocessingProto_.set_mean_file("mean.png");
    tt::protobuf::writeProtoToTextFile(filename, preprocessingProto_);

    tt::PreprocessingProto preprocessingProto_read;
    tt::protobuf::readProtoFromTextFile(filename, preprocessingProto_read);

    filename = "./preprocessing-read.txt";
    tt::protobuf::writeProtoToTextFile(filename, preprocessingProto_read);
}

