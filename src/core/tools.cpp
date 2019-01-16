/**
 * @file tools.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 29, 2018
 */

#include <string>
#include <vector>

#include "lane_line/common.h"
#include "lane_line/config.h"

#include "lane_line/Curve.h"
#include "lane_line/Drawing.h"
#include "lane_line/FileSystem.h"
#include "lane_line/LaneLineDetector.h"

#include "lane_line/protobuf_util.h"

namespace tt
{


LaneLineDetectorProto setUp()
{
    tt::LaneLineDetectorProto param;

    //--------------------IPM--------------------
    auto ipm_param_proto = param.mutable_ipm_param();

    std::string ipm_filename = std::string(TEST_DATA_DIR) + "/" + "1-ipm.txt";
    ipm_filename = "/home/fangjun/Documents/tusimple/lane-detection-challenge-cvpr-2017/train/clips/0531/1492727625007576150/2018-06-25-11-16-48.txt";  // NOLINT
    if (!tt::FileSystem::fileExists(ipm_filename))
    {
        LOG(FATAL) << "File " << ipm_filename << " does not exist.";
        exit(-1);
    }
    tt::IPMParameters ipm_param;
    CHECK(ipm_param.fromYAMLFile(ipm_filename));
    ipm_param.toProto(*ipm_param_proto);

    //--------------------Segnet--------------------
    auto preprocessing_proto = param.mutable_preprocessing_param();
    preprocessing_proto->set_model_file(CNN_PROTO_FILENAME);
    preprocessing_proto->set_trained_file(CNN_TRAINED_FILENAME);
    preprocessing_proto->set_use_gpu(CNN_USE_GPU);
    preprocessing_proto->set_name("SEGNET");

    //--------------------Row scan narrow-----------
    auto narrowing_proto = param.mutable_narrowing_param();
    narrowing_proto->set_name("RowScan");

    auto row_scan_proto = narrowing_proto->mutable_row_scan_param();
    row_scan_proto->set_threshold(230);

    //--------------------points extraction-----------
    auto points_extraction_proto = param.mutable_points_ex_param();
    points_extraction_proto->set_max_x_dir_search(20);
    points_extraction_proto->set_max_y_dir_search(20);

    //--------------------curve fitting-----------
    auto curve_fitting_proto = param.mutable_curve_fitting_param();
    curve_fitting_proto->set_threshold(2);
    curve_fitting_proto->set_max_iterations(1000);
    curve_fitting_proto->set_confidence(0.98f);
    curve_fitting_proto->set_name("line");
    curve_fitting_proto->set_minimum_points(30);

    param.add_curve_similarity_thresholds(8);   // delta_theta (degree)
    param.add_curve_similarity_thresholds(15);  // delta_d (pixels)
    param.set_minimum_distance_between_curves(160);  // at least 160 pixels between two curves  // NOLINT

    return param;
}

void loadConfigFromFile(const std::string &filename,
                        LaneLineDetectorProto *param)
{
    protobuf::readProtoFromTextFile(filename, *param);
}

void saveConfigToFile(const std::string &filename,
                      const LaneLineDetectorProto &param)
{
    protobuf::writeProtoToTextFile(filename, param);
}

void checkConfig(const LaneLineDetectorProto &param)
{
    //========================================
    // Preprocessing parameters
    //----------------------------------------
    if (!FileSystem::fileExists(param.preprocessing_param().model_file()))
    {
        LOG(FATAL) << "The file "
                   << param.preprocessing_param().model_file()
                   << " does not exist!";
        exit(-1);
    }

    if (!FileSystem::fileExists(param.preprocessing_param().trained_file()))
    {
        LOG(FATAL) << "The file "
                   << param.preprocessing_param().trained_file()
                   << " does not exist!";
        exit(-1);
    }
}



}  // namespace tt
