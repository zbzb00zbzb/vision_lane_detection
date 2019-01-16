/**
 * @file DateTime.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 05, 2018
 */

#include <string>

#include "lane_line/DateTime.h"

#include "lane_line/common.h"

namespace tt
{

std::string DateTime::toString()
{
    time_t t = time(NULL);
    struct tm st;
    localtime_r(&t, &st);
    std::string res = cv::format("%4d-%02d-%02d-%02d-%02d-%02d",
                                 st.tm_year + 1900,
                                 st.tm_mon + 1,
                                 st.tm_mday,
                                 st.tm_hour,
                                 st.tm_min,
                                 st.tm_sec);
    return res;
}


}  // namespace tt
