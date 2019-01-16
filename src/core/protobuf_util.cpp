/**
 * @file protobuf_util.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 06, 2018
 */

#include <stdio.h>

#include <google/protobuf/io/zero_copy_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>

#include <string>

#include "lane_line/protobuf_util.h"

namespace tt
{

namespace protobuf
{

void writeProtoToTextFile(const std::string &filename,
                          const google::protobuf::Message &message)     // NOLINT
{
    FILE *f = fopen(filename.c_str(), "w");
    if (!f)
    {
        LOG(FATAL) << "Failed to create file " << filename;
        return;
    }

    auto stream = new google::protobuf::io::FileOutputStream(fileno(f));
    CHECK_NOTNULL(stream);

    bool ret = google::protobuf::TextFormat::Print(message, stream);
    CHECK_EQ(ret, true) << "Failed to write " << filename;

    delete stream;

    fclose(f);
}

void readProtoFromTextFile(const std::string &filename,
                           google::protobuf::Message &message)      // NOLINT
{
    FILE *f = fopen(filename.c_str(), "r");
    if (!f)
    {
        LOG(FATAL) << "Failed to open file " << filename;
        return;
    }

    auto stream = new google::protobuf::io::FileInputStream(fileno(f));
    CHECK_NOTNULL(stream);

    bool ret = google::protobuf::TextFormat::Parse(stream, &message);
    CHECK_EQ(ret, true) << "Failed to read " << filename;

    delete stream;

    fclose(f);
}


std::string protoToString(const google::protobuf::Message &message)
{
    std::string str;
    google::protobuf::TextFormat::PrintToString(message, &str);
    return str;
}

}  // namespace protobuf
}  // namespace tt

