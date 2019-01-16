/**
 * @file
 * @author Kuang Fangjun <csukuangfj at gmail dot com>
 * @date September 06, 2018
 */

#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <string>

#include "lane_line/FileSystem.h"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "image_view_node");

    ros::NodeHandle nh;
    cv_bridge::CvImage cv_image;

    auto dir = "/home/fangjun/Desktop/yj";

    auto list_of_images = tt::FileSystem::getListOfFiles(dir, {".png"});
    ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/pylon_camera_node/image_raw", 1);   // NOLINT

    for (int i = 0; ; i++)
    {
        auto filename = list_of_images[i%list_of_images.size()];
        auto image = cv::imread(filename, cv::IMREAD_COLOR);
        cv_image.image = image;
        cv_image.encoding = "bgr8";
        sensor_msgs::Image ros_image;
        cv_image.toImageMsg(ros_image);
        pub.publish(ros_image);

        cv::flip(image, image, -1);
        cv::imshow("image", image);
        if ('q' == cv::waitKey(0))
        {
            break;
        }
    }
}
