/**
 * @file IPMTransformation.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 04, 2018
 */

#include "lane_line/IPMTransformation.h"

#include <algorithm>
#include <vector>

namespace tt
{

IPMTransformation::IPMTransformation(const IPMParametersProto &param)
    : param_(param)
{
    init(param_);
}

void IPMTransformation::init(const IPMParameters &param)
{
    IPMParametersProto proto;
    param.toProto(proto);

    init(proto);
}

void IPMTransformation::initWithPrecomputedH(const IPMParametersProto &param)
{
    CHECK(param.use_precomputed_h());
    CHECK_EQ(param.image_to_ipm_tf_size(), 9)
    << "A valid homography should have 9 elements! Current size: "
    << param.image_to_ipm_tf_size();

    param_ = param;

    LOG(INFO) << "size is: " << param_.image_to_ipm_tf_size();
    for (int i = 0; i < 9; i++)
    {
        image_to_ipm_.val[i] = param_.image_to_ipm_tf(i);
    }

    ipm_to_image_ = image_to_ipm_.inv();

    LOG(INFO) << "image to ipm: " << image_to_ipm_;
    LOG(INFO) << "ipm to image: " << ipm_to_image_;
}

void IPMTransformation::init(const IPMParametersProto &param)
{
    if (param.use_precomputed_h())
    {
        initWithPrecomputedH(param);
        return;
    }

    param_ = param;
    const float MY_PI = static_cast<float>(CV_PI);
    float c1 = cosf(param.pitch() * MY_PI / 180);
    float s1 = sinf(param.pitch() * MY_PI / 180);
    cv::Matx33f pitch_m(
            1, 0, 0,
            0, c1, s1,
            0, -s1, c1);

    float c2 = cosf(param.yaw() * MY_PI / 180);
    float s2 = sinf(param.yaw() * MY_PI / 180);
    cv::Matx33f yaw_m(
            c2, s2, 0,
            -s2, c2, 0,
            0, 0, 1);

    float c3 = cosf(param.roll() * MY_PI / 180);
    float s3 = sinf(param.roll() * MY_PI / 180);
    cv::Matx33f roll_m(
            c3, 0, -s3,
            0, 1, 0,
            s3, 0, c3);

    cv::Matx33f camera_to_world_m(
            1, 0, 0,
            0, 0, -1,
            0, 1, 0);

    cv::Matx33f image_to_camera_m(
            1.f / param.fx(), 0.f, -param.cx() / param.fx(),
            0.f, 1.f / param.fy(), -param.cy() / param.fy(),
            0.f, 0.f, 1.f);


    cv::Matx33f image_to_world_m = roll_m * yaw_m * pitch_m
                                   * camera_to_world_m * image_to_camera_m;

    cv::Point2f src_pts[4];
    cv::Point2f dst_pts[4];

    // 0-----1
    // |     |
    // |     |
    // 3-----2
    src_pts[0] = cv::Point2f(param.x1(), param.y1());
    src_pts[1] = cv::Point2f(param.x2(), param.y1());

    src_pts[2] = cv::Point2f(param.x2(), param.y2());
    src_pts[3] = cv::Point2f(param.x1(), param.y2());

    float min_x = INT_MAX;
    float max_x = INT_MIN;
    float min_y = INT_MAX;
    float max_y = INT_MIN;
    for (int i = 0; i < 4; ++i)
    {
        dst_pts[i] = transformPoint(image_to_world_m, src_pts[i]);

        min_x = std::min(min_x, dst_pts[i].x);
        max_x = std::max(max_x, dst_pts[i].x);
        min_y = std::min(min_y, dst_pts[i].y);
        max_y = std::max(max_y, dst_pts[i].y);
    }

    for (int i = 0; i < 4; ++i)
    {
        dst_pts[i].x = (dst_pts[i].x - min_x) / (max_x - min_x) * param.width();
        dst_pts[i].y = (dst_pts[i].y - min_y) / (max_y - min_y) * param.height();   // NOLINT
    }

    image_to_ipm_ = cv::getPerspectiveTransform(src_pts, dst_pts);
    ipm_to_image_ = image_to_ipm_.inv();

    // it is all 0s if the above statement fails
    CHECK_GT(cv::countNonZero(ipm_to_image_), 0)
        <<"Error in computing IPM transformation";
}

cv::Mat IPMTransformation::computeIPMImage(
        const cv::Mat &raw_image,
        int flags /*= cv::INTER_LINEAR*/) const
{
    cv::Mat ipm_image;
    cv::warpPerspective(raw_image,
                        ipm_image,
                        image_to_ipm_,
                        cv::Size(param_.width(), param_.height()),
                        flags);
    return ipm_image;
}

cv::Mat IPMTransformation::computeRawImage(
        const cv::Mat &ipm_image,
        const cv::Size &size) const
{
    cv::Mat raw_image;
    cv::warpPerspective(ipm_image, raw_image, ipm_to_image_, size);
    return raw_image;
}

std::shared_ptr<IPMTransformation>
IPMTransformation::create(const IPMParametersProto &param)
{
    auto res = std::make_shared<IPMTransformation>(param);

    return res;
}

}  // namespace tt
