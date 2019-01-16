/**
 * @file
 * @author Kuang Fangjun <csukuangfj at gmail dot com>
 * @date August 17, 2018
 *
 * Compute TP, FP and FN
 *
 * Usage
 *  1. ./test-lane-line to generate the ground truth
 *      - enable only EvaluationTest
 *      - the ground truth mask is saved to `write_mask_to_dir`
 *  2. ./general-main to generate the estimated results
 *      - the directory to save the estimated results are fixed in the code
 *  3. ./compute-metrics to compute the metrics
 */

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <string>
#include <utility>
#include <vector>

#include "lane_line/common.h"
#include "lane_line/config.h"
#include "lane_line/FileSystem.h"

namespace
{
std::string getName(const std::string &dir)
{
    auto p = dir.rfind("/");
    CHECK_NE(p, dir.npos);

    return dir.substr(p+1);
}
cv::Mat convertImage(const cv::Mat &image)
{
    cv::Mat res = cv::Mat::zeros(image.size(), CV_8U);
    const auto *pimage = image.ptr<uchar>(0);
    auto *pres = res.ptr<uchar>(0);
    for (int i = 0; i < image.total(); pres++, pimage++, i++)
    {
        if (*pimage)
        {
            *pres = 1;
        }
    }

    return res;
}

void compute(const cv::Mat &estimated_image, const cv::Mat &truth_image,
        float *TP, float *FN, float *FP)
{
    CHECK_EQ(estimated_image.size(), truth_image.size());
    CHECK_EQ(estimated_image.type(), truth_image.type());
    CHECK_EQ(estimated_image.type(), CV_8UC1);

    auto est = convertImage(estimated_image);
    auto truth = convertImage(truth_image);

    cv::Mat tp = est.mul(truth);
    tp = tp > 0;
    cv::Mat fp = est.mul(1 - truth);
    fp = fp > 0;
    cv::Mat fn = truth.mul(1 - est);
    fn = fn > 0;

    cv::imshow("estimated", estimated_image);
    cv::imshow("ground truth", truth_image);
    cv::imshow("TP", tp);
    cv::imshow("FP", fp);
    cv::imshow("FN", fn);
    if (cv::waitKey(0) == 'q') exit(0);

    *TP = cv::countNonZero(tp);
    *FP = cv::countNonZero(fp);
    *FN = cv::countNonZero(fn);
}

}  // namespace


int main(int, char *argv[])
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;

    std::string ground_truth_dir = "/home/fangjun/Desktop/results/compare/ground-truth";      // NOLINT
    std::string estimation_dir   = "/home/fangjun/Desktop/results/compare/estimated";         // NOLINT

    // <estimated, truth>
    std::vector<std::pair<std::string, std::string>> files;

    CHECK(tt::FileSystem::directoryExists(ground_truth_dir))
    << "Directory " << ground_truth_dir << " does not exist!";

    CHECK(tt::FileSystem::directoryExists(estimation_dir))
    << "Directory " << estimation_dir << " does not exist!";

    auto estimated = tt::FileSystem::getListOfFiles(estimation_dir, {".png"});
    for (const auto &f : estimated)
    {
        auto name = getName(f);
        name = ground_truth_dir + "/" + name;
        if (tt::FileSystem::fileExists(name))
        {
            files.emplace_back(f, name);
        }
    }

    LOG(INFO) << "There are " << files.size() << " pairs";

    float iou = 0;
    float total = 0;

    float all_tp = 0;
    float all_fp = 0;
    float all_fn = 0;
    for (size_t i = 0; i < files.size(); i++)
    {
        LOG(INFO) << "Read " << (i+1) << "/"
                  << files.size() << "." << files[i].first;
        auto estimated_image = cv::imread(files[i].first, cv::IMREAD_GRAYSCALE);
        auto truth_image = cv::imread(files[i].second, cv::IMREAD_GRAYSCALE);

        float TP, FP, FN;
        compute(estimated_image, truth_image, &TP, &FP, &FN);
        all_tp += TP;
        all_fp += FP;
        all_fn += FN;
        iou += TP / (TP + FP + FN);
        total++;
    }

    float precision = all_tp / (all_tp + all_fp);
    float recall = all_tp / (all_tp + all_fn);

    LOG(INFO) << "mean IoU: " << iou/total * 100 << "%";
    LOG(INFO) << "precision: " << precision * 100 << "%";
    LOG(INFO) << "recall: " << recall * 100 << "%";

    return 0;
}

/*
 * current results:
# width 8:
I0906 10:07:25.101961 15989 compute_eval_metrics.cpp:126] Read 84/84.
I0906 10:07:25.132582 15989 compute_eval_metrics.cpp:142] mean IoU: 59.2535%
I0906 10:07:25.132663 15989 compute_eval_metrics.cpp:143] precision: 72.98%
I0906 10:07:25.132680 15989 compute_eval_metrics.cpp:144] recall: 74.1909%

# width 9:
I0906 10:26:48.628115 17315 compute_eval_metrics.cpp:126] Read 84/84.
I0906 10:26:48.661278 17315 compute_eval_metrics.cpp:142] mean IoU: 63.3096%
I0906 10:26:48.661358 17315 compute_eval_metrics.cpp:143] precision: 76.1859%
I0906 10:26:48.661375 17315 compute_eval_metrics.cpp:144] recall: 77.4428%

# width 10:
I0906 09:33:23.276487 13284 compute_eval_metrics.cpp:126] Read 84/84.
I0906 09:33:23.306633 13284 compute_eval_metrics.cpp:142] mean IoU: 63.3096%
I0906 09:33:23.306699 13284 compute_eval_metrics.cpp:143] precision: 76.1859%
I0906 09:33:23.306704 13284 compute_eval_metrics.cpp:144] recall: 77.4428%

# width 11:
I0906 10:31:53.899713 17997 compute_eval_metrics.cpp:126] Read 84/84.
I0906 10:31:53.930176 17997 compute_eval_metrics.cpp:142] mean IoU: 66.3734%
I0906 10:31:53.930218 17997 compute_eval_metrics.cpp:143] precision: 78.5024%
I0906 10:31:53.930225 17997 compute_eval_metrics.cpp:144] recall: 79.7652%

# width 12
I0906 09:46:31.880960 14465 compute_eval_metrics.cpp:126] Read 84/84.
I0906 09:46:31.914916 14465 compute_eval_metrics.cpp:142] mean IoU: 66.3734%
I0906 09:46:31.914993 14465 compute_eval_metrics.cpp:143] precision: 78.5024%
I0906 09:46:31.915007 14465 compute_eval_metrics.cpp:144] recall: 79.7652%

# width 13
I0906 10:39:09.683706 18619 compute_eval_metrics.cpp:126] Read 84/84.
I0906 10:39:10.074542 18619 compute_eval_metrics.cpp:142] mean IoU: 68.7469%
I0906 10:39:10.074589 18619 compute_eval_metrics.cpp:143] precision: 80.2176%
I0906 10:39:10.074595 18619 compute_eval_metrics.cpp:144] recall: 81.518%

# width 14
I0906 11:52:24.496119 24822 compute_eval_metrics.cpp:126] Read 84/84.
I0906 11:52:24.529208 24822 compute_eval_metrics.cpp:142] mean IoU: 68.7469%
I0906 11:52:24.529294 24822 compute_eval_metrics.cpp:143] precision: 80.2176%
I0906 11:52:24.529309 24822 compute_eval_metrics.cpp:144] recall: 81.518%

# width 15
I0906 09:53:22.938179 15168 compute_eval_metrics.cpp:126] Read 84/84.
I0906 09:53:22.971117 15168 compute_eval_metrics.cpp:142] mean IoU: 61.7703%
I0906 09:53:22.971199 15168 compute_eval_metrics.cpp:143] precision: 66.5595%
I0906 09:53:22.971215 15168 compute_eval_metrics.cpp:144] recall: 88.3859%
*/
