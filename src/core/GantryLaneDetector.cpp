/**
 * @file GantryLaneDetector.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 25, 2018
 */
#include <algorithm>
#include <utility>
#include <vector>
#include <iostream>     // NOLINT

#include "lane_line/GantryLaneDetector.h"
#include "lane_line/Line.h"

#ifndef RAD_TO_DEG
#define RAD_TO_DEG(rad)  ((rad) * 180.f / static_cast<float>(CV_PI))
#endif

namespace
{

std::vector<tt::Point>
sortPoints(const std::vector<tt::Point> &input)
{
    std::vector<tt::Point> output(input);
    std::sort(output.begin(),
              output.end(),
              [](const tt::Point &first, const tt::Point &second)
              {return first.y() < second.y();});
    return output;
}

std::vector<std::vector<tt::Point> >
sortListOfPoints(const std::vector<std::vector<tt::Point> > &input)
{
    std::vector<std::vector<tt::Point> > output;
    for (const auto &p : input)
    {
        output.push_back(sortPoints(p));
    }
    return output;
}

std::vector<tt::Point>
samplePoints(const std::vector<tt::Point>& input, int step)
{
    std::vector<tt::Point> output;
    for (int i = 0; i < input.size(); i += step)
    {
        output.push_back(input[i]);
    }
    LOG(WARNING) << "input size: " << input.size();
    LOG(WARNING) << "step: " << step;
    LOG(WARNING) << "output size: " << output.size();
    return output;
}

std::vector<std::vector<tt::Point> >
sampleListOfPoints(const std::vector<std::vector<tt::Point> >& input, int step)
{
    std::vector<std::vector<tt::Point> > output;
    for (int i = 0; i < input.size(); i++)
    {
        output.push_back(samplePoints(input[i], step));
    }
    return output;
}

std::vector<float> removeMaxElement(const std::vector<float>& input)
{
    if (input.size() < 2)
    {
        return input;
    }

    std::vector<float> output = input;

    int i = 0;
    float max_val = input[0];
    for (int k = 1; k < input.size(); k++)
    {
        if (input[k] > max_val)
        {
            i = k;
            max_val = input[k];
        }
    }

    output.erase(output.begin() + i);
    return output;
}

std::vector<float> removeMinElement(const std::vector<float>& input)
{
    if (input.size() < 2)
    {
        return input;
    }

    std::vector<float> output = input;

    int i = 0;
    float min_val = input[0];
    for (int k = 1; k < input.size(); k++)
    {
        if (input[k] < min_val)
        {
            i = k;
            min_val = input[k];
        }
    }

    output.erase(output.begin() + i);
    return output;
}

/**
 * @param input
 * @return [0, 2pi)
 */
float computeAverageThetaInRadian(const std::vector<float> &input_arg)
{
    std::vector<float> input = input_arg;
    input = removeMaxElement(input);
    input = removeMinElement(input);
    LOG(INFO) << "input size: " << input_arg.size();
    LOG(INFO) << "after removing max/min: " << input.size();

    double res = 0;
    double s = 0;
    double c = 0;
    for (const auto theta : input)
    {
        s += std::sin(theta);
        c += std::cos(theta);
    }

    auto n = input.size();
    if (n)
    {
        s /= n;
        c /= n;
    }
    res = std::atan2(s, c);
    res = (res < 0) ? (res + 2*CV_PI) : res;

    return static_cast<float>(res);
}

float computeAverageThetaInRadian(
        const std::vector<std::pair<size_t, float> > &input)
{
    double res = 0;
    double s = 0;
    double c = 0;
    double weight = 0;
    for (const auto theta : input)
    {
        s += std::sin(theta.second) * theta.first;
        c += std::cos(theta.second) * theta.first;
        weight += theta.first;
    }

    if (weight)
    {
        s /= weight;
        c /= weight;
    }
    res = std::atan2(s, c);
    res = (res < 0) ? (res + 2*CV_PI) : res;

    return static_cast<float>(res);
}


/**
 *
 * @param angle
 * @return angle in [0, 2pi)
 */
float normalizeAngle2pi(float angle)
{
    angle = std::fmod(angle, static_cast<float>(CV_2PI));
    if (angle < 0)
    {
        angle += static_cast<float>(CV_2PI);
    }

    return angle;
}

/**
 *
 * @param angle
 * @return [-pi, pi)
 */
float normalizeAngle(float angle)
{
    angle = std::fmod(angle, static_cast<float>(CV_2PI));
    if (angle > static_cast<float>(CV_PI))
    {
        angle -= static_cast<float>(CV_2PI);
    }
    else if (angle < -static_cast<float>(CV_PI))    // NOLINT
    {
        angle += static_cast<float>(CV_2PI);
    }

    return angle;
}

/**
 *
 * @param angles
 * @return [0, 2pi)
 */
std::vector<float> normalizeAngles2pi(const std::vector<float>& angles)
{
    std::vector<float> res;
    for (const auto angle : angles)
    {
        auto a = normalizeAngle2pi(angle);
        res.emplace_back(a);
    }

    return res;
}

std::vector<std::pair<size_t, float>> normalizeAngles2pi(
        const std::vector<std::pair<size_t, float> >& angles)
{
    std::vector<std::pair<size_t, float> > res;
    for (const auto angle : angles)
    {
        auto a = normalizeAngle2pi(angle.second);
        res.emplace_back(angle.first, a);
    }

    return res;
}

/**
 *
 * @param angles
 * @return  [-pi, pi)
 */
std::vector<float> normalizeAngles(const std::vector<float>& angles)
{
    std::vector<float> res;
    for (const auto angle : angles)
    {
        auto a = normalizeAngle(angle);
        res.emplace_back(a);
    }

    std::stringstream ss;
    ss << "\nInput: \n";
    for (const auto a : angles)
    {
        ss << RAD_TO_DEG(a) << " ";
    }

    ss << ", average: " << RAD_TO_DEG(computeAverageThetaInRadian(angles));
    ss << "\nOutput: ";
    for (const auto a : res)
    {
        ss << RAD_TO_DEG(a) << " ";
    }
    ss << ", average: " << RAD_TO_DEG(computeAverageThetaInRadian(res));
    ss << "\n----end\n";
    LOG(INFO) << ss.str();

    return res;
}

std::vector<std::pair<size_t, float> > normalizeAngles(
        const std::vector<std::pair<size_t, float> >& angles)
{
    std::vector<std::pair<size_t, float> > res;
    for (const auto angle : angles)
    {
        auto a = normalizeAngle(angle.second);
        res.emplace_back(angle.first, a);
    }

    std::stringstream ss;
    ss << "\nInput: \n";
    for (const auto a : angles)
    {
        ss << RAD_TO_DEG(a.second) << " ";
    }

    ss << ", average: " << RAD_TO_DEG(computeAverageThetaInRadian(angles));
    ss << "\nOutput: ";
    for (const auto a : res)
    {
        ss << RAD_TO_DEG(a.second) << " ";
    }
    ss << ", weighted average: "
       << RAD_TO_DEG(computeAverageThetaInRadian(res));
    ss << "\n----end\n";
    LOG(INFO) << ss.str();

    return res;
}


#if 0
float averageAnglesInRadian(const std::vector<float>& angles)
{
    float sum = 0;
    for (const auto a : angles)
    {
        sum += a;
    }

    if (angles.size())
    {
        sum /= angles.size();
    }

    return sum;
}
#endif



}  // namespace

namespace tt
{
GantryLaneDetector::GantryLaneDetector(const LaneLineDetectorProto &param)
    : LaneLineDetector(param)
{
    CHECK_EQ(param.preprocessing_param().lane_type(), 3);
}

std::vector<std::shared_ptr<Curve> > GantryLaneDetector::detectCurves(
        const cv::Mat &raw_image)
{
    ipm_image_ = ipm_tf_->computeIPMImage(raw_image);

    preprocessing_->doPreprocessing(ipm_image_);
    auto preprocessing_result = preprocessing()->getResult();

    narrowing_->doNarrowing(preprocessing_result);
    auto narrowing_result     = narrowing_->getResult();

    points_extraction_->doExtraction(narrowing_result);
    auto points = points_extraction_->getPoints();
    std::cout << "initial line number: " << points.size() << std::endl;

    auto sorted_points = sortListOfPoints(points);
    auto sampled_points = sampleListOfPoints(sorted_points,
                                             param().sample_points_step_size());

    auto detected_curves = curve_fitting_->fitCurves(sampled_points);
    std::cout << "detect curves: " << detected_curves.size() << std::endl;

    detected_curves = Curve::sortCurves(detected_curves);

    // auto curves = fixDirections(detected_curves);
    auto curves = filtering(detected_curves);
    std::cout << "filtering curves: " << curves.size() << std::endl;

    detected_line_queue_.push_back(detected_curves);
    if (detected_line_queue_.size() > param().detected_line_queue_size())
    {
        detected_line_queue_.pop_front();
    }

    return curves;
}

std::vector<std::shared_ptr<Curve> > GantryLaneDetector::filtering(
        const std::vector<std::shared_ptr<Curve> > &curves)
{
    std::vector<std::shared_ptr<Curve> > output;
#if 0
    for (const auto &p : curves)
    {
        output.push_back(p->createCopy());
    }
#endif

    const float *thresholds =
            param().curve_similarity_thresholds().data();

    const int thresholds_size =
            param().curve_similarity_thresholds().size();

    for (const auto &c : curves)
    {
        std::vector<std::shared_ptr<Curve> > history;

        for (auto it = detected_line_queue_.crbegin();
             it != detected_line_queue_.crend(); it++)
        {
            for (const auto& q : *it)
            {
                if (c->isSimilarTo(q, thresholds, thresholds_size))
                {
                    history.emplace_back(q);
                    break;  // go to next frame
                }
            }
        }
        LOG(INFO) << "Current: " << c->toString();
        std::stringstream ss;

        ss << "History: \n";
        for (const auto& h : history)
        {
            ss << h->toString();
        }
        ss << "\n=====\n";
        LOG(INFO) << ss.str();
        std::vector<float> angles;

        angles.emplace_back(dynamic_cast<const Line*>(c.get())->getThetaInRadian());  // NOLINT
        for (const auto &h : history)
        {
            CHECK_EQ(h->type(), "line");
            const auto *line = dynamic_cast<const Line*>(h.get());
            angles.emplace_back(line->getThetaInRadian());
        }
        if (history.size() < param().minimum_detected_count())
        {
            LOG(ERROR) << "skip it: \n"
                       << "current size: " << history.size() << "\n"
                       << "required size: " << param().minimum_detected_count();
            continue;
        }

        auto avg = computeAverageThetaInRadian(angles);
        LOG(INFO) << "average: " << RAD_TO_DEG(avg);

        auto c_copy = c->createCopy();
        Line* line = dynamic_cast<Line*>(c_copy.get());
        line->setThetaInRadian(avg);
        output.push_back(c_copy);
    }

    return output;
}

std::vector<std::shared_ptr<Curve> > GantryLaneDetector::fixDirections(
        const std::vector<std::shared_ptr<Curve> > &curves)
{
    std::stringstream ss;
    std::vector<float> angles;
    std::vector<std::pair<size_t, float> > index_angles;
    ss << "\n----\nDirections\n----\n";
    for (const auto &c : curves)
    {
        CHECK_EQ(c->type(), "line");
        const auto line = dynamic_cast<const Line*>(c.get());
        ss << "degree:   " << line->getThetaInDegree() << "\n"
           << "inliners: " << line->inliers().size() << "\n"
           << "d:        " << line->getD() << "\n"
           << "----------------------------------\n";
        angles.emplace_back(line->getThetaInRadian());

        index_angles.emplace_back(line->inliers().size(),
                                  line->getThetaInRadian());
    }
    LOG(INFO) << ss.str();

    angles = normalizeAngles(angles);
    index_angles = normalizeAngles(index_angles);

    auto average = computeAverageThetaInRadian(angles);
    // average = computeAverageThetaInRadian(index_angles);

    std::vector<std::shared_ptr<Curve> > output = curves;
    for (auto &c : output)
    {
        auto* line = dynamic_cast<Line*>(c.get());
        // line->setThetaInRadian(average);
    }

    return output;
}


}  // namespace tt
