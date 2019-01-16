/**
 * @file ros_node.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 20, 2018
 */

#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>
#include <ros/ros.h>

#include <algorithm>
#include <string>
#include <vector>

#include "lane_line/common.h"
#include "lane_line/config.h"

#include "lane_line/DateTime.h"
#include "lane_line/Drawing.h"
#include "lane_line/FileSystem.h"
#include "lane_line/LaneLineDetector.h"
#include "lane_line/Line.h"
#include "lane_line/visualization.h"

#include "hadmap_msgs/Map.h"

namespace
{
/*
 * The mirror features of the camera is disabled.
 * See the manual page 341, table 59 for the available encodings of the camera.
 *
 * CompressedImage
 *  - format: bayer_rggb8; jpge compressed
 *
 * after decoding
 *   - height:   1200
 *   - rows:     1920
 *   - channels: 1
 *   - CV_8U
 *
 * color conversion (cv::COLOR_BayerBG2BGR):
 *
 */

class Wrapper
{
 public:
    Wrapper(const tt::LaneLineDetectorProto &param, ros::NodeHandle *nh)
        : nh_(*nh)
    {
        detector_ = tt::LaneLineDetector::create(param);
        CHECK(detector_);

        CHECK(nh->getParam("camera_topic", camera_topic_));
        CHECK(nh->getParam("hadmap_message_topic", hadmap_message_topic_));
        CHECK(nh->getParam("visualization_message_topic",
                          visualization_message_topic_));

        CHECK(nh->getParam("truck_pos_x", truck_pos_.x));
        CHECK(nh->getParam("truck_pos_y", truck_pos_.y));

        CHECK(nh->getParam("num_pixels_per_meter_x_dir",
                           num_pixels_per_meter_x_dir_));

        CHECK(nh->getParam("num_pixels_per_meter_y_dir",
                           num_pixels_per_meter_y_dir_));

        ROS_INFO_STREAM("camera_topic: " << camera_topic_);
        ROS_INFO_STREAM("hadmap_message_topic: " << hadmap_message_topic_);
        ROS_INFO_STREAM("visualization_message_topic: "
                                << visualization_message_topic_);
        ROS_INFO_STREAM("truck_pos_x: " << truck_pos_.x);
        ROS_INFO_STREAM("truck_pos_y: " << truck_pos_.y);
        ROS_INFO_STREAM("num_pixels_per_meter_x_dir_: "
                                << num_pixels_per_meter_x_dir_);
        ROS_INFO_STREAM("num_pixels_per_meter_y_dir_: "
                                << num_pixels_per_meter_y_dir_);

        auto pos = camera_topic_.find("compressed");
        if (pos != std::string::npos)
        {
            camera_sub_ = nh->subscribe(camera_topic_, 1,
                                        &Wrapper::compressedImageCallback,
                                        this);
        }
        else    // NOLINT
        {
            camera_sub_ = nh->subscribe(camera_topic_, 1,
                                        &Wrapper::imageCallback,
                                        this);
        }


        hadmap_pub_ = nh->advertise<hadmap_msgs::Map>(hadmap_message_topic_, 5);
        visualization_pub_ = nh->advertise<sensor_msgs::Image>(
                visualization_message_topic_, 5);
    }

    void processImage(const cv::Mat &raw, const std_msgs::Header &header)
    {
        cv::TickMeter tm;
        tm.start();
        auto curves = detector_->detectCurves(raw);
        std::cout << "detected curves num: " << curves.size() << std::endl;
        tm.stop();

        ROS_INFO_STREAM("\nstart----\n"
                                << "  " << tm.getTimeSec() << "s, "
                                << 1./tm.getTimeSec() << " Hz"
                                << "\nend---");

        auto draw_boundary = detector_->param().visualization_param().draw_left_right_boundary();    // NOLINT
        auto only_show_original = detector_->param().visualization_param().only_show_original_image();     // NOLINT

        auto vis = tt::visualizeResult(detector_, raw, draw_boundary, only_show_original);  // NOLINT

        cv_bridge::CvImage cv_image;
        cv_image.header.stamp = header.stamp;
        cv_image.encoding = "bgr8";
        cv_image.image = vis;

        auto ros_image_ptr = cv_image.toImageMsg();
        visualization_pub_.publish(*ros_image_ptr);

        publisheDetectedCurves();
    }

    void imageCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        cv::Mat image = cv_bridge::toCvCopy(msg)->image;
        cv::cvtColor(image, image, cv::COLOR_BayerBG2BGR);
        // for tianjing port. because the camera is mounted as bottom-up
        // cv::flip(raw, raw, 0);  // vertical
        // cv::flip(raw, raw, 1);  // horizontal
        //cv::flip(image, image, -1);    // both
        // std::cout << "flip image param: " << detector_->getFlipImage() << std::endl;
        if(detector_->getFlipImage())
        {
            cv::flip(image, image, -1);
        }
        if (!detector_->ipm_tf()->param().use_precomputed_h())
        {
            // if H is not precomputed, we resize it to 960x600
            cv::resize(image, image, cv::Size(960, 600));
        }
        processImage(image, msg->header);
    }


    void compressedImageCallback(
            const sensor_msgs::CompressedImageConstPtr &msg)
    {
        cv::Mat raw = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_UNCHANGED);
        cv::cvtColor(raw, raw, cv::COLOR_BayerBG2BGR);

        // for tianjing port. because the camera is mounted as bottom-up
        // cv::flip(raw, raw, 0);  // vertical
        // cv::flip(raw, raw, 1);  // horizontal
        //cv::flip(raw, raw, -1);    // both
        std::cout << "flip image param: " << detector_->getFlipImage() << std::endl;
        if(!detector_->getFlipImage())
        {
            cv::flip(raw, raw, -1);
        }

        if (!detector_->ipm_tf()->param().use_precomputed_h())
        {
            // if H is not precomputed, we resize it to 960x600
            cv::resize(raw, raw, cv::Size(960, 600));
        }
        processImage(raw, msg->header);
    }

 private:
    void publisheDetectedCurves()
    {
        auto &curves = detector_->getKalmanFilteredCenterCurves();
        if (curves.size() < 2)
        {
            LOG(ERROR) << "Only " << curves.size() << " detected! Skip it";
            return;
        }
        if (curves[0]->type() != "line")
        {
            LOG(FATAL) << "Not implemented for: " << curves[0]->type();
            return;
        }

        static cv::Point2f truck_pos = tt::Transformation::transformPoint(
                detector_->ipm_tf()->getImageToIPM(),
                truck_pos_);

        auto sorted_curves = tt::Curve::sortCurves(curves);

        auto doTransform = [this, &truck_pos](const cv::Point2f &input)
        {
            cv::Point2f p;
            p = input - truck_pos;
            p.x /= -1 * num_pixels_per_meter_x_dir_;
            p.y /= -1 * num_pixels_per_meter_y_dir_;
            std::swap(p.x, p.y);

            hadmap_msgs::Point res;
            res.point.x = p.x;
            res.point.y = p.y;

            return res;
        };

        hadmap_msgs::Map map;
        hadmap_msgs::Section section;

        int n = static_cast<int>(sorted_curves.size());
        auto center_curves = detector_->getKalmanFilteredCenterCurves();
        auto left_curves = detector_->getKalmanFilteredLeftCurves();
        auto right_curves = detector_->getKalmanFilteredRightCurves();  
        
        for (int i = 0; i < n - 1; i++)
        {
            hadmap_msgs::Lane lane;
            std::vector<tt::Point> points;
            const auto *lane_left_line = right_curves[i]->getLinePtr();
            CHECK_NOTNULL(lane_left_line);
            lane_left_line->computeStartEndPoints(detector_->getIpmImage(), points);
            bool left_line_valid = false;
            if ((points[0].x() == -1 && points[0].y() == -1) || (points[1].x() == -1 && points[1].y() == -1))
            {
               left_line_valid = false; 
            }
            else{
                left_line_valid = true;
            }
            //end point is image bottom, points[1] is end point
            //right_curve is left line of lane; left_curve is right line of lane
            //hadmap coordinate is x forward and y right
            auto p_start_left = doTransform(points[0]);
            auto p_end_left = doTransform(points[1]);

            const auto *lane_right_line = left_curves[i+1]->getLinePtr();
            CHECK_NOTNULL(lane_right_line);
            lane_right_line->computeStartEndPoints(detector_->getIpmImage(), points);
            bool right_line_valid = false;
            if ((points[0].x() == -1 && points[0].y() == -1) || (points[1].x() == -1 && points[1].y() == -1))
            {
               right_line_valid = false; 
            }
            else{
                right_line_valid = true;
            }
            auto p_start_right = doTransform(points[0]);
            auto p_end_right = doTransform(points[1]);
	    
            bool lane_section_flag = false;
            if(p_end_left.point.y >= 0.0 && p_end_left.point.y < 3. && left_line_valid){
                hadmap_msgs::Point point;
                std::cout << "left_line_y: " << p_end_left.point.y << std::endl; 
                point.point.x = p_end_left.point.x;
                point.point.y = p_end_left.point.y; 
		        lane.pts_left.push_back(point);
                point.point.x = p_start_left.point.x;
                point.point.y = p_start_left.point.y;
                lane.pts_left.push_back(point);
                lane_section_flag = true;
	        }
            if(p_end_right.point.y < 0.0 && p_end_right.point.y > -3. && right_line_valid){
                hadmap_msgs::Point point;
                std::cout << "right_line_y: " << p_end_right.point.y << std::endl;
                point.point.x = p_end_right.point.x;
                point.point.y = p_end_right.point.y; 
                lane.pts_right.push_back(point);
                point.point.x = p_start_right.point.x;
                point.point.y = p_start_right.point.y;
                lane.pts_right.push_back(point);
                lane_section_flag = true;
            }
	        if(lane_section_flag){
            	section.lanes.push_back(lane);
                break;
            }
        }

        std_msgs::Header header;
        header.stamp = ros::Time::now();
        map.header = header;
        map.sections.push_back(section);
        hadmap_pub_.publish(map);
    }

 private:
    std::shared_ptr<tt::LaneLineDetector> detector_;

    std::string camera_topic_;
    std::string hadmap_message_topic_;
    std::string visualization_message_topic_;

    cv::Point truck_pos_;

    float num_pixels_per_meter_x_dir_;
    float num_pixels_per_meter_y_dir_;

    ros::NodeHandle &nh_;
    ros::Subscriber camera_sub_;
    ros::Publisher hadmap_pub_;
    ros::Publisher visualization_pub_;
};

}  // namespace

int main(int argc, char* argv[])
{
    google::InitGoogleLogging(argv[0]);

    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;

#if IS_PX2
    ros::init(argc, argv, "lane_line_px2");
#else
    ros::init(argc, argv, "lane_line");
#endif

    ros::NodeHandle nh("~");

    ROS_INFO_STREAM("Started!");

    std::string config_filename;
    if (!nh.getParam("proto_txt_config", config_filename))
    {
        LOG(ERROR) << "Failed to read config file from rosparam server!\n"
                   << "Use " << CONFIG_PROTO_FILENAME << " by default";
        config_filename = CONFIG_PROTO_FILENAME;
    }

    ROS_INFO_STREAM("Use config file: " << config_filename);
    CHECK(tt::FileSystem::fileExists(config_filename))
          << "File " << config_filename << " does not exist!";

    tt::LaneLineDetectorProto param;
    tt::loadConfigFromFile(config_filename, &param);

    if (!param.mutable_preprocessing_param()->has_model_file())
    {
        std::string proto_file;
        if (!nh.getParam("caffe_proto_file", proto_file))
        {
            proto_file = CNN_PROTO_FILENAME;
        }

        ROS_INFO_STREAM("Use caffe proto file: " << proto_file);

        CHECK(tt::FileSystem::fileExists(proto_file))
                << "File " << proto_file << " does not exist!";

        *(param.mutable_preprocessing_param()->mutable_model_file())
                = proto_file;
    }

    if (!param.mutable_preprocessing_param()->has_trained_file())
    {
        std::string trained_file;
        if (!nh.getParam("caffe_trained_file", trained_file))
        {
            trained_file = CNN_TRAINED_FILENAME;
        }

        ROS_INFO_STREAM("Use caffe trained file: " << trained_file);

        CHECK(tt::FileSystem::fileExists(trained_file))
        << "File " << trained_file << " does not exist!";

        *(param.mutable_preprocessing_param()->mutable_trained_file())
                = trained_file;
    }

    tt::checkConfig(param);
    std::string filename = std::string("/tmp/current-config-") +
                           tt::DateTime::toString()+ ".txt";
    tt::saveConfigToFile(filename, param);
    tt::saveConfigToFile("/tmp/latest-config.txt", param);

    Wrapper wrapper(param, &nh);

    ros::spin();

    return 0;
}
