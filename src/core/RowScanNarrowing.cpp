/**
 * @file RowScanNarrowing.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 07, 2018
 */

#include <utility>  // for std::pair
#include <vector>

#include "lane_line/RowScanNarrowing.h"

#include "lane_line/common.h"

namespace
{


enum class StateRowScan
{
    START,
    ADD,
};

/**
 *
 * @param data a list of (index, value)
 * @return
 */
int computeWeightedMean(const std::vector<std::pair<int, int> > &data)
{
    float sum    = 0;
    float weight = 0;

    for (const auto &p : data)
    {
        sum += p.first * p.second;
        weight += p.second;
    }

    return cvRound(sum/weight);
}


}  // namespace

namespace tt
{

RowScanNarrowing::RowScanNarrowing(const NarrowingProto &param)
    : Narrowing(param),
      threshold_(param.row_scan_param().threshold()),
      minimum_width_(param.row_scan_param().minimum_width())

{
    CHECK_GT(threshold_, 1);
    CHECK_GE(minimum_width_, 2);
}


// compute the weighed mean of the position
void RowScanNarrowing::doNarrowing(const cv::Mat &image)
{
    CHECK(image.type() == CV_8UC1) << "Incorrect image format!";

    std::vector<std::pair<int, int> > data;  // (index, intensity)

    int height = image.rows;
    int width = image.cols;

    result_ = cv::Mat::zeros(image.size(), CV_32SC3);  // (left_x, 255, right_x)

    StateRowScan state;
    for (int j = 0; j < height; j++)
    {
        const auto p = image.ptr<uchar>(j);
        auto presult = result_.ptr<cv::Vec3i>(j);

        state = StateRowScan::START;

        for (int i = 0; i < width; i++)
        {
            switch (state)
            {
                case StateRowScan::START:
                    if (p[i] > threshold_)
                    {
                        data.clear();
                        data.emplace_back(i, p[i]);
                        state = StateRowScan::ADD;
                    }
                    break;
                case StateRowScan::ADD:
                    if (p[i] > threshold_)
                    {
                        data.emplace_back(i, p[i]);
                    }
                    else  // NOLINT
                    {
                        if (data.size() >= minimum_width_)
                        {
                            int center = computeWeightedMean(data);
                            presult[center] = cv::Vec3i(data.front().first,
                                                        255,
                                                        data.back().first);
                        }

                        state = StateRowScan::START;
                    }

                    break;
            }  // end switch (state)

            if ((i == width-1) && state == StateRowScan::ADD)
            {
                if (data.size() >= minimum_width_)
                {
                    int center = computeWeightedMean(data);
                    presult[center] = cv::Vec3i(data.front().first,
                                                255,
                                                data.back().first);
                }
            }
        }  // for (int i = 0; i < width; i++)
    }  // for (int j = 0; j < height; j++)
}

/*
 * To display the result, use
 * @code
        auto narrowing_result = getResult();
        cv::Mat mv[3];
        cv::split(narrowing_result, mv);
        mv[1].convertTo(mv[1], CV_8U);
        cv::imshow("result", mv[1]);
        cv::waitkey(0);
 * @endcoe
 */
cv::Mat RowScanNarrowing::getResult() const
{
    return result_;
}


}  // namespace tt
