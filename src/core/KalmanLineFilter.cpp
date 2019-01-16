/**
 * @file KalmanLineFilter.h
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date August 08, 2018
 *
 */
#include <limits>
#include <vector>

#include "lane_line/KalmanLineFilter.h"
#include "lane_line/Line.h"

namespace tt
{

KalmanLineFilter::KalmanLineFilter(const KalmanLineFilterProto &param)
{
    CHECK_GE(param.minimum_theta_diff_in_degree(), 0);
    CHECK_GE(param.minimum_d_diff_in_pixel(), 0);
    CHECK_GE(param.maximum_missed_count(), 1); // if the missed count of a tracked line is larger than this value,          // NOLINT
                                               // this tracked line is not tracked any more and is removed from the map.    // NOLINT
    param_ = param;
}

std::shared_ptr<KalmanLineFilter> KalmanLineFilter::create(
        const KalmanLineFilterProto &param)
{
    std::shared_ptr<KalmanLineFilter> res;
    res.reset(new KalmanLineFilter(param));

    CHECK_NOTNULL(res.get());

    return res;
}

void KalmanLineFilter::doKalmanLineFilter(
        std::vector<std::shared_ptr<Curve>> *curves)
{
    if (!curves)
    {
        return;
    }

    for (auto &p : detected_lines_)
    {
        p.second.missed_count_++;
    }

    for (auto &c : *curves)
    {
        auto *line = c->getLinePtr();
        CHECK_NOTNULL(line);
        int id = wasDetected(*line);
        if (detected_lines_.count(id))
        {
            // the line was detected before
            predict(id);
            update(id, *line);
            getState(id, line);
        }
        else    // NOLINT
        {
            // a new line
            createTrackedLines(*line);
        }
    }

    // remove lines that have not been detected for more than the given count.
    removeLines();
}

void KalmanLineFilter::removeLines()
{
    std::vector<int> id;
    for (const auto &p : detected_lines_)
    {
        if (p.second.missed_count_ > param_.maximum_missed_count())
        {
            id.emplace_back(p.first);
        }
    }

    for (auto i : id)
    {
        detected_lines_.erase(i);
    }
}

int KalmanLineFilter::wasDetected(const Line &line) const
{
    int res = -1;
    for (const auto &p : detected_lines_)
    {
        float theta_in_degree = p.second.kalman_filter_.statePost.at<float>(0, 0);  // NOLINT
        float d_in_pixel = p.second.kalman_filter_.statePost.at<float>(1, 0);

        float theta_diff = fabsf(theta_in_degree - line.getThetaInDegree());
        if (theta_diff > 180)
        {
            theta_diff = fabsf(360 - theta_diff);
        }

        float d_diff = fabsf(d_in_pixel - line.getD());

        if ((theta_diff < param_.minimum_theta_diff_in_degree()) &&
                (d_diff < param_.minimum_d_diff_in_pixel()))
        {
            res = p.first;
            break;
        }
    }

    return res;
}

int KalmanLineFilter::getNextAvailableId() const
{
    int res = -1;
    for (int i = 0; i < std::numeric_limits<int>::max(); i++)
    {
        if (detected_lines_.count(i))
        {
            continue;
        }
        res = i;
        break;
    }

    if (res < 0)
    {
        LOG(FATAL) << "No more ID is available!";
    }

    return res;
}

void KalmanLineFilter::createTrackedLines(const Line &line)
{
    // track
    //      0. theta_in_degree
    //      1. d_in_pixel
    // of a line
    int id = getNextAvailableId();
    auto &kalman = detected_lines_[id].kalman_filter_;

    int num_state = 2;
    int num_measurement = 2;
    int num_control_input = 0;
    kalman.init(num_state, num_measurement, num_control_input, CV_32F);

    kalman.statePost.at<float>(0, 0) = line.getThetaInDegree();
    if (line.getThetaInDegree() > 180)
    {
        kalman.statePost.at<float>(0, 0) -= 360;    // we assume that theta is near 0, e.g., 350 -- 10      NOLINT
    }

    kalman.statePost.at<float>(1, 0) = line.getD();

    // var(theta_in_degree)
    kalman.errorCovPost.at<float>(0, 0) = 1;

    kalman.errorCovPost.at<float>(0, 1) = 0;
    kalman.errorCovPost.at<float>(1, 0) = 0;

    // var(d_in_pixel)
    kalman.errorCovPost.at<float>(1, 1) = 1;

    cv::setIdentity(kalman.transitionMatrix);
    kalman.processNoiseCov.at<float>(0, 0) = 1;
    kalman.processNoiseCov.at<float>(0, 1) = 0;
    kalman.processNoiseCov.at<float>(1, 0) = 0;
    kalman.processNoiseCov.at<float>(1, 1) = 1;

    cv::setIdentity(kalman.measurementMatrix);
    cv::setIdentity(kalman.measurementNoiseCov);
    kalman.measurementNoiseCov.at<float>(1, 1) = 5;
}

void KalmanLineFilter::predict(int id)
{
    CHECK_EQ(detected_lines_.count(id), 1)
        << "No tracked line has the id: " << id;

    detected_lines_[id].missed_count_ = 0;
    detected_lines_[id].kalman_filter_.predict();

    auto& theta = detected_lines_[id].kalman_filter_.statePre.at<float>(0, 0);
    if (theta > 180)
    {
        theta -= 360;
        detected_lines_[id].kalman_filter_.statePost.at<float>(0, 0) -= 360;
    }
    else if (theta < -180)  // NOLINT
    {
        theta += 360;
        detected_lines_[id].kalman_filter_.statePost.at<float>(0, 0) += 360;
    }
}

void KalmanLineFilter::update(int id, const Line &line)
{
    CHECK_EQ(detected_lines_.count(id), 1)
        << "No tracked line has the id: " << id;

    cv::Mat measurement(2, 1, CV_32FC1);

    measurement.at<float>(0, 0) = line.getThetaInDegree();
    if (line.getThetaInDegree() > 180)
    {
        measurement.at<float>(0, 0) -= 360;
    }
    else if (line.getThetaInDegree() < -180)    // NOLINT
    {
        measurement.at<float>(0, 0) += 360;
    }

    measurement.at<float>(1, 0) = line.getD();

    detected_lines_[id].kalman_filter_.correct(measurement);
    auto &theta = detected_lines_[id].kalman_filter_.statePost.at<float>(0, 0);
    if (theta > 180)
    {
        theta -= 360;
    }
    else if (theta < -180)  // NOLINT
    {
        theta += 360;
    }
}

void KalmanLineFilter::getState(int id, Line *line)
{
    CHECK_EQ(detected_lines_.count(id), 1)
        << "No tracked line has the id: " << id;
    CHECK_NOTNULL(line);

    auto &state = detected_lines_[id].kalman_filter_.statePost;
    float &theta_in_degree = state.at<float>(0, 0);
    float d_in_pixel = state.at<float>(1, 0);
    if (theta_in_degree < 0)
    {
        theta_in_degree += 360;
    }

    if (theta_in_degree >= 360)
    {
        theta_in_degree -= 360;
    }

    line->initWithDegree(theta_in_degree, d_in_pixel);
}

}  // namespace tt
