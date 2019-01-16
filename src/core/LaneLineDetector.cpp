/**
 * @file LaneLineDetector.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 07, 2018
 *
 * Pipeline:
 *  1. IPM transformation to get the IPM image
 *
 *  2. CNN segmentation taking the IPM image as input (preprocessing)
 *
 *  3. RowScan to thin the output of preprocessing
 *      - parameters:
 *          - threshold: pixels with intensity less than this value are ignored
 *          - minimum width: area width less than this value is ignored.
 *
 *  4. Extract points for fitting lines (or curves)
 *      - parameters:
 *          - max_x_dir_search
 *          - max_y_dir_search
 *
 *  5. Fit lines with RANSAC
 *
 *  6. Merge lines
 *
 *  7. Remove lines
 */

#include <algorithm>
#include <iostream>     // NOLINT
#include <memory>
#include <string>
#include <vector>

#include "lane_line/GantryLaneDetector.h"
#include "lane_line/LaneLineDetector.h"
#include "lane_line/Line.h"

namespace
{

cv::Mat getGradientMask(const cv::Mat &ipm)
{
    cv::Mat gray;

    cv::cvtColor(ipm, gray, cv::COLOR_BGR2GRAY);

    gray.convertTo(gray, CV_32F);

    int lane_width  = 15;

    int nr = gray.rows;
    int nc = gray.cols;

    cv::Mat grad = cv::Mat::zeros(gray.size(), CV_8UC1);

    for (int r = 0; r < nr; r++)
    {
        const auto *p = gray.ptr<float>(r);
        auto *pgrad = grad.ptr<uchar>(r);
        for (int c = lane_width; c < nc - lane_width; c++)
        {
            if (p[c] < 50) continue;
            auto diff = 2*p[c] - p[c - lane_width] - p[c + lane_width];
            if (diff > 50)
            {
                pgrad[c] = 1;
            }
        }
    }

    return grad;
}

}  // namespace

namespace tt
{

std::shared_ptr<LaneLineDetector> LaneLineDetector::create(
        const LaneLineDetectorProto &param)
{
    CHECK(param.has_type());

    std::shared_ptr<LaneLineDetector> res;

    if (param.type() == "gantry")
    {
        res.reset(new GantryLaneDetector(param));
    }
    else if (param.type() == "general")     // NOLINT
    {
        res.reset(new LaneLineDetector(param));
    }
    else  // NOLINT
    {
        LOG(FATAL) << "Unknown detector type: "
                   << param.type()
                   << "\nSupported types are: \n"
                   << " gantry, general";
    }

    return res;
}

std::string LaneLineDetector::type() const
{
    return "general";
}

LaneLineDetector::LaneLineDetector(const LaneLineDetectorProto &param)
{
    init(param);
}

void LaneLineDetector::init(const LaneLineDetectorProto &param)
{
    param_ = param;

    CHECK_GT(param_.detected_line_queue_size(), 0) << "Invalid queue size!";

    curve_fitting_ = CurveFitting::create(param.curve_fitting_param());

    flip_image_ = param.flip_image();

    ipm_tf_        = IPMTransformation::create(param.ipm_param());
    preprocessing_ = Preprocessing::create(param_.preprocessing_param());
    narrowing_     = Narrowing::create(param.narrowing_param());
    points_extraction_ = PointsExtraction::create(param.points_ex_param());

    merge_lines_ = MergeLines::create(param.merge_lines_param(),
                                      param.curve_fitting_param());

    remove_lines_ = RemoveLines::create(param.remove_lines_param());

    filter_lines_ = FilterLines::create(param.filter_lines_param());

    kalman_center_line_filter_ = KalmanLineFilter::create(
            param.kalman_line_filter_param());

    kalman_left_line_filter_ = KalmanLineFilter::create(
            param.kalman_line_filter_param());

    kalman_right_line_filter_ = KalmanLineFilter::create(
            param.kalman_line_filter_param());
    
    img_queue_size_ = 9;
}

void LaneLineDetector::sortLines(
        std::vector<std::shared_ptr<tt::Curve> > *curves)
{
    if (curves->empty()) return;

    for (const auto &c : *curves)
    {
        CHECK_EQ(c->type(), "line");
    }

    std::sort(curves->begin(), curves->end(),
              [](const std::shared_ptr<tt::Curve> &left,
                 const std::shared_ptr<tt::Curve> &right)
              {
                  return left->getLinePtr()->getD() < right->getLinePtr()->getD();  // NOLINT
              });
}

std::vector<std::shared_ptr<Curve> >
LaneLineDetector::detectCurves(const cv::Mat &raw_image)
{
    cv::TickMeter total_time;
    cv::TickMeter post_time;

    total_time.start();
    post_time.start();

    cv::TickMeter ipm_time;

    ipm_time.start();
    ipm_image_ = ipm_tf_->computeIPMImage(raw_image);
    ipm_time.stop();

    post_time.stop();

    LOG(INFO) << " do preprocessing";
    cv::TickMeter preprocessing_time;
    preprocessing_time.start();
    preprocessing_->doPreprocessing(ipm_image_);
    auto preprocessing_result = preprocessing()->getResult();
    
    if (img_queue_.size() < img_queue_size_)
    {
        img_queue_.push_back(preprocessing_result);
        std::vector<std::shared_ptr<Curve> > curves;
        return curves;
    }
    else{
        img_queue_.erase(img_queue_.begin());
        img_queue_.push_back(preprocessing_result);
        cv::Mat tmp = (img_queue_[4] + img_queue_[6] + img_queue_[8]);
        preprocessing_result = tmp.clone();
    }
    // cv::imwrite("/home/xd/1.png", preprocessing_result);
    
    preprocessing_time.stop();

    post_time.start();

    if (param_.preprocessing_param().use_gradient_mask())
    {
        auto grad = getGradientMask(ipm_image_);
        preprocessing_result = preprocessing_result.mul(grad);
    }

    cv::TickMeter narrowing_time;

    LOG(INFO) << " do narrowing";
    narrowing_time.start();
    narrowing_->doNarrowing(preprocessing_result);
    auto narrowing_result = narrowing_->getResult();
    narrowing_time.stop();

    cv::TickMeter points_ex_time;

    LOG(INFO) << " do points extraction";
    points_ex_time.start();
    points_extraction_->doExtraction(narrowing_result);
    auto points = points_extraction_->getPoints();
    // std::cout << "initial line number: " << points.size() << std::endl;

    points_ex_time.stop();

    LOG(INFO) << " do fitting";
    cv::TickMeter fit_curve_time;
    fit_curve_time.start();
    auto detected_curves = curve_fitting_->fitCurves(points);
    fit_curve_time.stop();

    cv::TickMeter merge_curve_time;
    merge_curve_time.start();
    merge_lines_->doMerge(detected_curves);
    auto merged_curves = merge_lines_->getResult();
    merge_curve_time.stop();

    cv::TickMeter remove_curve_time;
    remove_curve_time.start();
    remove_lines_->doRemove(merged_curves);

    auto removed_curves = remove_lines_->getResult();
    remove_curve_time.stop();

    center_curves_ = filter_lines_->filterLines(removed_curves);
    sortLines(&center_curves_);

    left_curves_ = getLeftBoundary(center_curves_);
    right_curves_ = getRightBoundary(center_curves_);

    kalman_center_line_filter_->doKalmanLineFilter(&center_curves_);
    kalman_left_line_filter_->doKalmanLineFilter(&left_curves_);
    kalman_right_line_filter_->doKalmanLineFilter(&right_curves_);

#if 0
    cv::TickMeter missed_curve_time;
    missed_curve_time.start();
    auto missed_curves = fillInMissingCurves(removed_curves);
    missed_curve_time.stop();

    post_time.stop();
    total_time.stop();

    queue().push_back(missed_curves);
    if (queue().size() > param_.detected_line_queue_size())
    {
        queue().pop_front();
    }
#endif

    auto curves = detected_curves;
    curves = merged_curves;
    curves = removed_curves;
    curves = center_curves_;

#if 0
    std::cout << "ipm time:                  "
              << ipm_time.getTimeMilli() << " ms, "
              <<  1./ipm_time.getTimeSec() << " Hz\n";

    std::cout << "cnn time:                  "
              << preprocessing_time.getTimeMilli() << " ms, "
              <<  1./preprocessing_time.getTimeSec() << " Hz\n";

    std::cout << "narrowing time:            "
              << narrowing_time.getTimeMilli() << " ms, "
              <<  1./narrowing_time.getTimeSec() << " Hz\n";

    std::cout << "point extraction time:     "
              << points_ex_time.getTimeMilli() << " ms, "
              <<  1./points_ex_time.getTimeSec() << " Hz\n";

    std::cout << "curve fit time:            "
              << fit_curve_time.getTimeMilli() << " ms, "
              <<  1./fit_curve_time.getTimeSec() << " Hz\n";

    std::cout << "curve merge time:            "
              << merge_curve_time.getTimeMilli() << " ms, "
              <<  1./merge_curve_time.getTimeSec() << " Hz\n";

    std::cout << "curve remove time:            "
              << remove_curve_time.getTimeMilli() << " ms, "
              <<  1./remove_curve_time.getTimeSec() << " Hz\n";

    std::cout << "curve fill time:            "
              << missed_curve_time.getTimeMilli() << " ms, "
              <<  1./missed_curve_time.getTimeSec() << " Hz\n";

    std::cout << "post processing time:            "
              << post_time.getTimeMilli() << " ms, "
              <<  1./post_time.getTimeSec() << " Hz\n";

    std::cout << "total time:            "
              << total_time.getTimeMilli() << " ms, "
              <<  1./total_time.getTimeSec() << " Hz\n";
#endif
    return curves;
}

std::vector<std::shared_ptr<Curve> >
LaneLineDetector::mergeCurves(
        const std::vector<std::shared_ptr<Curve> > &curves)
{
    std::vector<std::shared_ptr<Curve> > output;
    for (const auto &p : curves)
    {
        output.push_back(p->createCopy());
    }

    const float *thresholds =
            param().curve_similarity_thresholds().data();

    const int thresholds_size =
            param().curve_similarity_thresholds().size();

    bool anything_merged = true;
    while (anything_merged)
    {
        anything_merged = false;
        for (size_t i = 0; i < output.size(); i++)
        {
            auto& this_curve = output[i];
            for (size_t j = i+1; j < output.size(); j++)
            {
                auto other_curve = output[j];
                bool is_similar = this_curve->isSimilarTo(other_curve,
                                                          thresholds,
                                                          thresholds_size);
                if (is_similar)
                {
                    auto this_str = this_curve->toString();
                    std::vector<Point> points;
                    points.insert(points.end(),
                                  this_curve->inliers().begin(),
                                  this_curve->inliers().end());

                    points.insert(points.end(),
                                  other_curve->inliers().begin(),
                                  other_curve->inliers().end());

                    this_curve = curve_fitting_->fitCurve(points);
                    LOG(WARNING) << "\n" << "this:" << this_str << "\n"
                              << "other:" << other_curve->toString() << "\n"
                              << "after merging:\n" << this_curve->toString()
                              << "\n***********\n";


                    output.erase(output.begin() + j);
                    j--;
                    anything_merged = true;
                }
            }
        }
    }

    return output;
}

std::vector<std::shared_ptr<Curve> >
LaneLineDetector::removeInvalidCurves(
        const std::vector<std::shared_ptr<Curve> > &curves)
{
    std::vector<std::shared_ptr<Curve> > output;
    for (const auto &p : curves)
    {
        output.push_back(p->createCopy());
    }

    bool anything_removed = true;
    while (anything_removed)
    {
        anything_removed = false;
        for (size_t i = 0; i < output.size(); i++)
        {
            for (size_t j = i+1; j < output.size(); j++)
            {
                float diff = output[i]->absDistanceToThisCurve(output[j]);
                if (diff < param().minimum_distance_between_curves())
                {
                    if (output[j]->inliers().size() >
                            output[i]->inliers().size())
                    {
                        std::swap(output[i], output[j]);
                    }

                    output.erase(output.begin() + j);
                    j--;
                    anything_removed = true;
                }
            }
        }
    }

    return output;
}

std::vector<std::shared_ptr<Curve> > LaneLineDetector::fillInMissingCurves(
        const std::vector<std::shared_ptr<Curve> > &curves)
{
    std::vector<std::shared_ptr<Curve> > output;
    for (const auto &p : curves)
    {
        output.push_back(p->createCopy());
    }

    std::vector<std::shared_ptr<Curve> > missed;

    auto curve_exists = [this](const std::shared_ptr<Curve> &curve,
                           const std::vector<std::shared_ptr<Curve> > curves)
    {
        bool res = false;
        for (const auto& c : curves)
        {
            if (c->isSimilarTo(curve,
                            this->param().curve_similarity_thresholds().data(),
                            this->param().curve_similarity_thresholds().size()))
            {
                res = true;
                break;
            }
        }
        return res;
    };

    for (auto it = detected_line_queue_.crbegin();
         it != detected_line_queue_.crend(); it++)
    {
        for (const auto qc : *it)
        {
            bool exists = curve_exists(qc, missed) || curve_exists(qc, output);

            if (exists) continue;

            if (qc->missed_num() && qc->getConfidence() < 0.5)
            {
                continue;  // do not track this lane any more
            }

            auto curve = qc->createCopy();
            curve->missed_num() = qc->missed_num() + 1;

            missed.push_back(curve);
        }
    }

    output.insert(output.begin(), missed.begin(), missed.end());

    return output;
}

std::vector<std::shared_ptr<Curve> > LaneLineDetector::getLeftBoundary(
        const std::vector<std::shared_ptr<Curve> > &curves) const
{
    std::vector<std::shared_ptr<Curve> > res;
    for (const auto &c : curves)
    {
        auto points = c->getLeftBoundary();
        auto left = curve_fitting_->fitCurve(points);
        res.push_back(left);
    }

    return res;
}

std::vector<std::shared_ptr<Curve> > LaneLineDetector::getRightBoundary(
        const std::vector<std::shared_ptr<Curve> > &curves) const
{
    std::vector<std::shared_ptr<Curve> > res;
    for (const auto &c : curves)
    {
        auto points = c->getRightBoundary();
        auto right = curve_fitting_->fitCurve(points);
        res.push_back(right);
    }

    return res;
}

void LaneLineDetector::showDebugInfo()
{
    std::cout << "-------------------------------------------" << std::endl;
    std::cout << "------------------debug info---------------" << std::endl;
    std::cout << "-------------------------------------------" << std::endl;
    std::cout << "---------------end debug info--------------" << std::endl;
    std::cout << "-------------------------------------------" << std::endl;
}


}  // namespace tt
