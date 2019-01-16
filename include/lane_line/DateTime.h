/**
 * @file DateTime.h
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 04, 2018
 */

#ifndef LANE_LINE_DATETIME_H
#define LANE_LINE_DATETIME_H

#include <string>

namespace tt
{


class DateTime
{
 public:
    /**
     *  yyyy-mm-dd-hh-mm-ss
     *
     * @return e.g.,  2018-06-05-14-41-44
     */
    static std::string toString();
};


}  // namespace tt


#endif  // LANE_LINE_DATETIME_H
