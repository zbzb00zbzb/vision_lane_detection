/**
 * @file Evaluation.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date August 16, 2018
 *
 */

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <string>
#include <vector>

#include "lane_line/Drawing.h"
#include "lane_line/Evaluation.h"
#include "lane_line/FileSystem.h"

namespace
{
cv::Mat getLaneMarkingMask(
        const cv::Mat &image,
        const tt::ColorProto &lane_marking_color)
{
    CHECK_EQ(image.type(), CV_8UC3) << "It accepts only 3-channel rgb image";
    cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1);

    auto total = image.total();

    const auto *pimage = image.ptr<cv::Vec3b>(0, 0);
    auto *pmask= mask.ptr<uchar>(0);
    for (size_t i = 0; i < total; i++, pimage++, pmask++)
    {
        if (pimage[0][0] != lane_marking_color.blue()) continue;
        if (pimage[0][1] != lane_marking_color.green()) continue;
        if (pimage[0][2] != lane_marking_color.red()) continue;

        pmask[0] = 255;
    }

    return mask;
}

std::string getBaseName(const std::string &name)
{
    auto pos = name.rfind("/");
    CHECK_NE(pos, name.npos);

    return name.substr(pos+1);
}

}  // namespace

namespace tt
{

Evaluation::Evaluation(const tt::EvaluationProto &param)
{
    std::string error_info;
    CHECK(checkParam(param, &error_info)) << error_info;

    if (!FileSystem::directoryExists(param.write_mask_to_dir()))
    {
        LOG(INFO) << "Create directory " << param.write_mask_to_dir();
        FileSystem::createDirectory(param.write_mask_to_dir());
        if (!FileSystem::directoryExists(param.write_mask_to_dir()))
        {
            LOG(FATAL) << "Failed to create: " << param.write_mask_to_dir();
        }
    }

    CHECK_GT(param.draw_width(), 0) << "drawn width should be larger than 0";

    param_ = param;
    ipm_tf_ = IPMTransformation::create(param_.ipm_param());

    narrowing_     = Narrowing::create(param.narrowing_param());
    points_extraction_ = PointsExtraction::create(param.points_ex_param());
    curve_fitting_ = CurveFitting::create(param.curve_fitting_param());

    merge_lines_ = MergeLines::create(param.merge_lines_param(),
                                      param.curve_fitting_param());

    remove_lines_ = RemoveLines::create(param.remove_lines_param());
}

std::shared_ptr<Evaluation> Evaluation::create(
        const tt::EvaluationProto &param)
{
    std::shared_ptr<Evaluation> res;
    res.reset(new Evaluation(param));
    CHECK_NOTNULL(res.get());
    return res;
}


bool Evaluation::checkParam(
        const tt::EvaluationProto &param,
        std::string *error_str) const
{
    // image mask dir should exist
    bool res = true;
    std::ostringstream ss;
    if (!param.image_mask_dir_size())
    {
        ss << "No image mask dir!";
    }
    else        // NOLINT
    {
        for (int i = 0; i < param.image_mask_dir_size(); i++)
        {
            const auto &dir = param.image_mask_dir(i);
            if (!FileSystem::directoryExists(dir))
            {
                ss << "Directory " << dir << " does not exist!\n";
                res  = false;
            }
        }
    }

    if (!res && error_str)
    {
        *error_str = ss.str();
    }

    return res;
}

void Evaluation::generateListOfMasks()
{
    mask_filename_.clear();
    for (int i = 0; i < param_.image_mask_dir_size(); i++)
    {
        const auto &dir = param_.image_mask_dir(i);
        auto list_of_images = FileSystem::getListOfFiles(dir, {".png"});
        mask_filename_.insert(mask_filename_.end(),
                list_of_images.begin(), list_of_images.end());
    }
}

void Evaluation::generateIpmMasks()
{
    generateListOfMasks();
    for (const auto &s : mask_filename_)
    {
        auto image = cv::imread(s, cv::IMREAD_COLOR);
        auto mask = getLaneMarkingMask(image, param_.lane_marking_color());

        cv::resize(image, image, cv::Size(960, 600), cv::INTER_NEAREST);
        cv::resize(mask, mask, cv::Size(960, 600), cv::INTER_NEAREST);

        auto ipm_mask = ipm_tf_->computeIPMImage(mask);

        ipm_masks_[s] = ipm_mask;
    }
}

void Evaluation::generateLines()
{
    for (const auto &p : ipm_masks_)
    {
        narrowing_->doNarrowing(p.second);
        auto narrowing_result     = narrowing_->getResult();
        cv::Mat mv[3];
        cv::split(narrowing_result, mv);
        mv[1].convertTo(mv[1], CV_8U);

        points_extraction_->doExtraction(narrowing_result);
        auto points = points_extraction_->getPoints();

        auto curves = curve_fitting_->fitCurves(points);

        merge_lines_->doMerge(curves);
        curves = merge_lines_->getResult();
        remove_lines_->doRemove(curves);

        curves = remove_lines_->getResult();

        cv::Mat image = cv::Mat::zeros(p.second.size(), CV_8UC3);
        for (const auto &c : curves)
        {
            auto *line = c->getLinePtr();
            CHECK_NOTNULL(line);

            tt::Drawing::lineInPlace(image,
                                     *line,
                                     tt::Drawing::white(), param_.draw_width());
        }
        cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
        image = image > 100;


        cv::imshow("ipm", p.second);
        cv::imshow("image", image);
        cv::imshow("narrowing", mv[1]);

        auto filename = param().write_mask_to_dir()+"/"+getBaseName(p.first);
        cv::imwrite(filename, image);
        LOG(INFO) << "write to " << filename;
        if (cv::waitKey(0) == 'q')
        {
            break;
        }
    }
}

}  // namespace tt

