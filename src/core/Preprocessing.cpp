/**
 * @file Preprocessing.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 06, 2018
 */

#include "lane_line/common.h"

#include "lane_line/FileSystem.h"
#include "lane_line/Preprocessing.h"
#include "lane_line/SegnetPreprocessing.h"

namespace tt
{

Preprocessing::Preprocessing(const PreprocessingProto &param)
    : param_(param)
{
    CHECK(FileSystem::fileExists(param_.model_file()));
    CHECK(FileSystem::fileExists(param_.trained_file()));
    if (!param.mean_file().empty())
    {
        CHECK(FileSystem::fileExists(param_.mean_file()));
    }
}

std::shared_ptr<Preprocessing>
Preprocessing::create(const PreprocessingProto &param)
{
    std::shared_ptr<Preprocessing> res;
    auto name = param.name();
    if (name == "SEGNET")
    {
        res = std::make_shared<SegnetPreprocessing>(param);
    }
    else    // NOLINT
    {
        LOG(FATAL) << "Unknown preprocessing name: " << name;
    }
    return res;
}


}  // namespace tt
