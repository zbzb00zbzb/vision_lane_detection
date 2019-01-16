/**
 * @file protobuf_util.h
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 06, 2018
 */

#ifndef LANE_LINE_PROTOBUF_UTIL_H
#define LANE_LINE_PROTOBUF_UTIL_H

#include <string>

#include "lane_line/common.h"

namespace tt
{
namespace protobuf
{

void writeProtoToTextFile(const std::string &filename,
                          const google::protobuf::Message &message);    // NOLINT

void readProtoFromTextFile(const std::string &filename,
                           google::protobuf::Message &message);         // NOLINT

std::string protoToString(const google::protobuf::Message &message);

}  // namespace protobuf
}  // namespace tt

#endif  // LANE_LINE_PROTOBUF_UTIL_H
