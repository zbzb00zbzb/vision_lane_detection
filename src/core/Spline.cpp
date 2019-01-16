/**
 * @file Spline.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date July 02, 2018
 */

#include <string>
#include <vector>

#include "lane_line/common.h"
#include "lane_line/Spline.h"

namespace tt
{

void Spline::compute(const std::vector<Point> &points)
{
    CHECK_GT(points.size(), 2)
        << "It needs at least three points for a cubic spline.";

    int n = static_cast<int>(points.size());
    n--;  //  number of segments

    std::vector<float> a(n+1);
    for (int i = 0; i <= n; i++)
    {
        a[i] = points[i].y();
    }

    //====================
    //     step 1
    //--------------------
    std::vector<float> h(n);
    for (int i = 0; i < n; i++)
    {
        h[i] = points[i+1].x() - points[i].x();
    }

    //====================
    //     step 2
    //--------------------
    std::vector<float> alpha(n);        // alpha[0] is not used
    for (int i = 1; i < n; i++)
    {
        alpha[i] = 3 / h[i] * (a[i+1] - a[i]) - 3 / h[i-1] * (a[i] - a[i-1]);
    }

    //====================
    //     step 3
    //--------------------
    std::vector<float> l(n+1, 0);
    std::vector<float> mu(n+1, 0);
    std::vector<float> z(n+1, 0);
    l[0] = 1;

    //====================
    //     step 4
    //--------------------
    for (int i = 1; i < n; i++)
    {
        l[i] = 2*(points[i+1].x() - points[i-1].x()) - h[i-1] * mu[i-1];
        mu[i] = h[i] / l[i];
        z[i] = (alpha[i] - h[i-1] * z[i-1]) / l[i];
    }

    //====================
    //     step 5
    //--------------------
    l[n] = 1;
    z[n] = 0;
    std::vector<float> c(n+1, 0);

    //====================
    //     step 6
    //--------------------
    std::vector<float> b(n, 0);
    std::vector<float> d(n, 0);
    for (int j = n-1; j >= 0; j--)
    {
        c[j] = z[j] - mu[j] * c[j+1];
        b[j] = (a[j+1] - a[j])/h[j] - h[j]*(c[j+1] + 2*c[j])/3;
        d[j] = (c[j+1] - c[j]) / (3*h[j]);
    }


    //====================
    //     step 7
    //--------------------
    segments_.clear();
    segments_.resize(n);
    for (int j = 0; j < n; j++)
    {
        segments_[j].a_ = a[j];
        segments_[j].b_ = b[j];
        segments_[j].c_ = c[j];
        segments_[j].d_ = d[j];
        segments_[j].x_ = points[j].x();
        segments_[j].step_ = points[j+1].x() - points[j].x();
    }
}

std::vector<Point> Spline::interpolate(const std::vector<Point> &points,
                                       int num /* = 20*/)
{
    CHECK_GE(num, 0);
    if (segments_.empty())
    {
        compute(points);
    }

    if (num == 0)
    {
        return points;
    }

    std::vector<Point> res;

    for (const auto &seg : segments_)
    {
        float delta_x = seg.step_ / num;
        for (int i = 0; i < num; i++)
        {
            float x = seg.x_ + delta_x * i;
            float y = seg.compute(x);
            res.emplace_back(x, y);
        }
    }

    return res;
}

std::string Spline::toString() const
{
    std::stringstream ss;
    int n = static_cast<int>(segments_.size());
    for (int i = 0; i < n; i++)
    {
        ss << "\n-----Segment " << i << "-----\n";
        ss << " a: " << segments_[i].a_ << "\n";
        ss << " b: " << segments_[i].b_ << "\n";
        ss << " c: " << segments_[i].c_ << "\n";
        ss << " d: " << segments_[i].d_ << "\n";
        ss << " x: " << segments_[i].x_ << "\n";
    }

    ss << "-----end-----\n";

    return ss.str();
}

}  // namespace tt
