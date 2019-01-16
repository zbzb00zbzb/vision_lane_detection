/**
 * @file Evaluation.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date August 16, 2018
 */

#include <gtest/gtest.h>

#include <string>

#include "lane_line/Evaluation.h"
#include "lane_line/protobuf_util.h"

namespace tt
{



class EvaluationTest : public ::testing::Test
{
 public:
    void SetUp() override
    {
        std::string config_filename = EVAL_PROTO_FILENAME;

        EvaluationProto param;
        protobuf::readProtoFromTextFile(config_filename, param);
        eval_ = Evaluation::create(param);
    }

 protected:
    std::shared_ptr<Evaluation> eval_;
};

TEST_F(EvaluationTest, create)
{
    LOG(INFO) << eval_->param().DebugString();
}

TEST_F(EvaluationTest, generate_mask_filename)
{
    eval_->generateIpmMasks();
    eval_->generateLines();
}

}  // namespace tt
