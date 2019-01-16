/**
 * @file PointsExtraction.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 07, 2018
 *
 */

#include <vector>

#include "lane_line/PointsExtraction.h"

#include "lane_line/Point.h"

namespace
{

/**
 * Get the neighbors of the specified point.
 * @param point
 * @param flag If a pixel is 1, it is skipped.
 * @param image
 * @param nx
 * @param ny
 * @return
 */
std::vector<tt::Point> getNeighbors(const tt::Point &point,
                                    const cv::Mat &flag,
                                    const cv::Mat &image,
                                    int nx , int ny)
{
    CHECK(image.type() == CV_32SC3);
    CHECK(flag.type() == CV_8UC1);

    CHECK_EQ(image.size(), flag.size());

    int height = flag.rows;
    int width = flag.cols;

    CHECK_LT(point.x(), width) << "Invalid point";
    CHECK_LT(point.y(), height) << "Invalid point";

    CHECK_GE(point.x(), 0) << "Invalid point";
    CHECK_GE(point.y(), 0) << "Invalid point";

    std::vector<tt::Point> neighbors;

    for (int j = point.y() - ny; j <= point.y() + ny; j++)
    {
        // it is outside, ignore it
        if (j < 0) continue;

        // it is outside ignore it
        if (j >= height) break;

        const auto pflag = flag.ptr<uchar>(j);
        const auto pimage = image.ptr<cv::Vec3i>(j);
        for (int i = point.x() - nx; i <= point.x() + nx; i++)
        {
            // outside, skip
            if (i < 0) continue;

            // outside, skip
            if (i >= width) break;

            // current pixel, skip
            if ((j == point.y()) && (i == point.x())) continue;

            // the pixel has already been visited, skip
            if (pflag[i] > 0) continue;

            // this is a black pixel, skip
            if (!pimage[i][1]) continue;

            tt::Point pp(i, j);
            pp.setLeftX(pimage[i][0]);
            pp.setRightX(pimage[i][2]);

            neighbors.push_back(pp);
        }
    }

    return neighbors;
}

}  // namespace

namespace tt
{

PointsExtraction::PointsExtraction(const PointsExtractionProto &param)
    : param_(param)
{
    CHECK_GT(param.max_x_dir_search(), 0) << "Invalid x search range";
    CHECK_GT(param.max_y_dir_search(), 0) << "Invalid y search range";

    max_x_dir_search_ = param.max_x_dir_search();
    max_y_dir_search_ = param.max_y_dir_search();
}

void PointsExtraction::doExtraction(const cv::Mat &image)
{
    points_.clear();
    CHECK(image.type() == CV_32SC3);

    cv::Mat flag = cv::Mat::zeros(image.size(), CV_8UC1);

    int height = image.rows;
    int width = image.cols;

    int nx = max_x_dir_search_;
    int ny = max_y_dir_search_;

    for (int j = height-1; j >= 0; j--)
    {
        const auto p = image.ptr<cv::Vec3i>(j);
        auto pf = flag.ptr<uchar>(j);
        for (int i = 0; i < width; i++)
        {
            // this pixel has been processed
            if (pf[i]) continue;

            // this pixel has 0 intensity, skip
            if (!p[i][1])
            {
                // set the flag to 1 also for an invalid pixel
                pf[i] = 1;
                continue;
            }

            std::vector<Point> stack;
            std::vector<Point> set;

            Point pp;
            pp.setXY(i, j);
            pp.setLeftX(p[i][0]);
            pp.setRightX(p[i][2]);

            stack.push_back(pp);

            while (!stack.empty())
            {
                auto current = stack.back();
                stack.pop_back();

                // this pixel has been processed, skip it
                if (flag.at<uchar>(cv::Point2i(current.x(), current.y())))
                {
                    continue;
                }

                CHECK(image.at<cv::Vec3i>(cv::Point2i(current.x(), current.y()))[1]);        // NOLINT
                CHECK(!flag.at<uchar>(cv::Point2i(current.x(), current.y())));

                set.push_back(current);
                flag.at<uchar>(cv::Point2i(current.x(), current.y())) = 1;

                auto tmp_neighbor = getNeighbors(current, flag, image, nx, ny);
                stack.insert(stack.end(), tmp_neighbor.begin(),
                             tmp_neighbor.end());
            }

            points_.push_back(set);
        }
    }
}

std::shared_ptr<PointsExtraction>
PointsExtraction::create(const PointsExtractionProto &param)
{
    (void)param;

    std::shared_ptr<PointsExtraction> res;

    res = std::make_shared<PointsExtraction>(param);

    return res;
}


}  // namespace tt
