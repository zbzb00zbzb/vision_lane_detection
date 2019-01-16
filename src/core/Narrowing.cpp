/**
 * @file Narrowing.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 07, 2018
 */


#include "lane_line/common.h"

#include "lane_line/DistTransNarrowing.h"
#include "lane_line/Narrowing.h"
#include "lane_line/RowScanNarrowing.h"

namespace tt
{

Narrowing::Narrowing(const NarrowingProto &param)
    : param_(param)
{}

std::shared_ptr<Narrowing> Narrowing::create(const NarrowingProto &param)
{
    std::shared_ptr<Narrowing> res;
    auto name = param.name();
    if (name == "RowScan")
    {
        res = std::make_shared<RowScanNarrowing>(param);
    }
    else if (name == "DistTrans")   // NOLINT
    {
        res = std::make_shared<DistTransNarrowing>(param);
    }
    else                            // NOLINT
    {
        LOG(FATAL) << "Unknown narrowing name: " << name;
    }

    return res;
}

}  // namespace tt
