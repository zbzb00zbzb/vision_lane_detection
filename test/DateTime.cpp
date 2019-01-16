/**
 * @file DateTime.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 05, 2018
 */

#include <gtest/gtest.h>

#include "lane_line/DateTime.h"

#include "lane_line/common.h"

class DateTimeTest : public ::testing::Test
{
 protected:
    tt::DateTime dt_;
};

TEST_F(DateTimeTest, to_string)
{
    auto str = dt_.toString();
    LOG(INFO) << str;
}

