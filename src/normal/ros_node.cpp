/**
 * @file ros_node.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date August 09, 2018
 */

#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>
#include <ros/ros.h>

#include <hadmap_msgs/Map.h>

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

        CHECK(nh->getParam("num_pixels_per_meter_x_dir",
                           num_pixels_per_meter_x_dir_));

        CHECK(nh->getParam("num_pixels_per_meter_y_dir",
                           num_pixels_per_meter_y_dir_));

        // TODO: (fangjun) support both compressed and uncompressed images
        camera_sub_ = nh->subscribe(camera_topic_, 1, &Wrapper::callback, this);
        hadmap_pub_ = nh->advertise<hadmap_msgs::Map>(hadmap_message_topic_, 5);
        visualization_pub_ = nh->advertise<sensor_msgs::Image>(
                visualization_message_topic_, 5);
    }

    void callback(const sensor_msgs::ImageConstPtr& msg)
    {
        cv::Mat raw = cv_bridge::toCvCopy(msg)->image;

        cv::cvtColor(raw, raw, cv::COLOR_BayerBG2BGR);
        // cv::flip(raw, raw, 0);  // vertical
        // cv::flip(raw, raw, 1);  // horizontal
        // cv::flip(raw, raw, -1);    // both

        if (!detector_->ipm_tf()->param().use_precomputed_h())
        {
            cv::resize(raw, raw, cv::Size(960, 600));
        }

        cv::TickMeter tm;
        tm.start();
        auto curves = detector_->detectCurves(raw);
        tm.stop();

        ROS_INFO_STREAM("\nstart----\n"
                                << "  " << tm.getTimeSec() << "s, "
                                << 1./tm.getTimeSec() << " Hz"
                                << "\nend---");
        auto draw_boundary = detector_->param().visualization_param().draw_left_right_boundary();    // NOLINT
        auto only_show_original = detector_->param().visualization_param().only_show_original_image();     // NOLINT
        auto vis = tt::visualizeResult(detector_, raw, draw_boundary, only_show_original);  // NOLINT

        cv_bridge::CvImage cv_image;
        cv_image.header.stamp = ros::Time::now();
        cv_image.encoding = "bgr8";
        cv_image.image = vis;

        auto ros_image_ptr = cv_image.toImageMsg();
        visualization_pub_.publish(*ros_image_ptr);

        publisheDetectedCurves();
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
            p.x *= -1 * num_pixels_per_meter_x_dir_;
            p.y *= -1 * num_pixels_per_meter_y_dir_;
            std::swap(p.x, p.y);

            hadmap_msgs::Point res;
            res.point.x = p.x;
            res.point.y = p.y;

            return res;
        };

        hadmap_msgs::Map map;
        hadmap_msgs::Section section;

        int n = static_cast<int>(sorted_curves.size());

        auto left_curves = detector_->getKalmanFilteredLeftCurves();
        auto right_curves = detector_->getKalmanFilteredRightCurves();

        left_curves = sorted_curves;
        right_curves = sorted_curves;

        std::vector<tt::Point> points;
        for (int i = 0; i < n - 1; i++)
        {
            hadmap_msgs::Lane lane;
            const auto *line = right_curves[i]->getLinePtr();
            CHECK_NOTNULL(line);
            line->computeStartEndPoints(detector_->getIpmImage(), points);

            auto p = doTransform(points[0]);
            lane.pts_left.push_back(p);

            p = doTransform(points[1]);
            lane.pts_left.push_back(p);

            line = left_curves[i+1]->getLinePtr();
            CHECK_NOTNULL(line);
            line->computeStartEndPoints(detector_->getIpmImage(), points);

            p = doTransform(points[0]);
            lane.pts_right.push_back(p);

            p = doTransform(points[1]);
            lane.pts_right.push_back(p);

            section.lanes.push_back(lane);
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
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;
    google::InitGoogleLogging(argv[0]);

#if IS_PX2
    ros::init(argc, argv, "lane_line_px2");
#else
    ros::init(argc, argv, "lane_line");
#endif

    ROS_INFO_STREAM("Started!");

    std::string config_filename = CONFIG_PROTO_FILENAME;
    tt::LaneLineDetectorProto param;
    tt::loadConfigFromFile(config_filename, &param);

    tt::checkConfig(param);
    std::string filename = std::string("/tmp/current-config-") +
                           tt::DateTime::toString()+ ".txt";
    tt::saveConfigToFile(filename, param);
    tt::saveConfigToFile("/tmp/latest-config.txt", param);

    ros::NodeHandle nh("~");
    Wrapper wrapper(param, &nh);

    ros::spin();

    return 0;
}

