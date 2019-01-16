/**
 * @author Kuang Fangjun <csukuangfj at gmail dot com>
 * @date August 13, 2018
 *
 * Show/Save images from ros topic.
 *
 * usage:
 *   roslaunch lane_line image_view.launch
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


std::string g_camera_topic = "/left/pylon_camera_node/image_raw";
int g_resize_image_to_width = -1;
int g_resize_image_to_height = -1;

bool g_flip_x = false;
bool g_flip_y = false;

bool g_show_image = true;

bool g_save_image = false;
std::string g_save_image_to_dir = "/home/fangjun/Desktop/images/";

bool g_save_video = false;
std::string g_save_video_to_filename = "/tmp/test.mp4";

cv::VideoWriter g_writer;

namespace
{

bool directoryExists(const std::string &directory_name)
{
    bool res = false;
    struct stat sb;
    if (!stat(directory_name.c_str(), &sb))
    {
        res = S_ISDIR(sb.st_mode);
    }

    return res;
}

bool createDirectory(const std::string &directory_name)
{
    if (directoryExists(directory_name)) return true;

    char buf[500] = {0};
    snprintf(buf, sizeof(buf), "mkdir -p %s", directory_name.c_str());

    int res = system(buf);
    return res == 0;
}

void processImage(const cv::Mat &_image, const std_msgs::Header &header)
{
    cv::Mat image = _image.clone();
    if (g_flip_y)
    {
        cv::flip(image, image, 0);
    }

    if (g_flip_x)
    {
        cv::flip(image, image, 1);
    }

    int height = image.rows;
    int width = image.cols;
    if (g_resize_image_to_width > 0)
    {
        width = g_resize_image_to_width;
    }

    if (g_resize_image_to_height > 0)
    {
        height = g_resize_image_to_height;
    }

    cv::resize(image, image, cv::Size(width, height));

    if (g_save_image)
    {
        std::ostringstream ss;
        ss << g_save_image_to_dir << "/" << header.stamp.toNSec() << ".jpg";
        cv::imwrite(ss.str(), image);
        static int i = 1;
        ROS_INFO_STREAM("Save " << i <<  " to " << ss.str() << "\n");
        i++;
    }

    if (g_show_image)
    {
        cv::namedWindow("image", cv::WINDOW_NORMAL);
        cv::imshow("image", image);
        if ('q' == cv::waitKey(1))
        {
            ros::shutdown();
        }
    }

    if (g_save_video)
    {
        g_writer << image;
    }
}

void imageCallback(
        const sensor_msgs::ImageConstPtr &_image)
{
    cv::Mat image = cv_bridge::toCvCopy(_image)->image;
    // cv::cvtColor(image, image, cv::COLOR_BayerBG2BGR);
    processImage(image, _image->header);
}

void compressedImageCallback(
        const sensor_msgs::CompressedImageConstPtr &_image)
{
    cv::Mat image = cv::imdecode(cv::Mat(_image->data), cv::IMREAD_UNCHANGED);
    cv::cvtColor(image, image, cv::COLOR_BayerBG2BGR);
    processImage(image, _image->header);
}

}  // namespace


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "image_view_node");

    ros::NodeHandle nh("~");

#define MY_READ(nh, param)                                  \
  if (!nh.getParam(#param, g_##param))                      \
  {                                                         \
    ROS_ERROR_STREAM("Failed to read: " << #param);         \
    exit(-1);                                               \
  }                                                         \
  else                                                      \
  {                                                         \
    ROS_INFO_STREAM("Read " #param << ": " << g_##param);   \
  }

    MY_READ(nh, camera_topic);

    MY_READ(nh, resize_image_to_width);
    MY_READ(nh, resize_image_to_height);

    MY_READ(nh, flip_x);
    MY_READ(nh, flip_y);

    MY_READ(nh, show_image);
    MY_READ(nh, save_image);
    MY_READ(nh, save_image_to_dir);

    MY_READ(nh, save_video);
    MY_READ(nh, save_video_to_filename);

#undef MY_READ

    if (g_save_image)
    {
        if (!directoryExists(g_save_image_to_dir))
        {
            ROS_INFO_STREAM("Creating directory: " << g_save_image_to_dir);
            createDirectory(g_save_image_to_dir);
            if (!directoryExists(g_save_image_to_dir))
            {
                ROS_ERROR_STREAM("Failed to create directory "
                                         << g_save_image_to_dir);
                exit(-1);
            }
        }
    }

    if (g_save_video)
    {
        int frame_rate = 20;
        g_writer.open(g_save_video_to_filename, CV_FOURCC('X', '2', '6', '4'),
                    frame_rate, cv::Size(960, 600));
        if (!g_writer.isOpened())
        {
            ROS_ERROR_STREAM("Failed to create: " << g_save_video_to_filename);
            exit(-1);
        }
    }

    ros::Subscriber sub;
    auto pos = g_camera_topic.find("compressed");
    if (pos != std::string::npos)
    {
        sub = nh.subscribe(g_camera_topic, 200, &compressedImageCallback);
    }
    else    // NOLINT
    {
        sub = nh.subscribe(g_camera_topic, 200, &imageCallback);
    }

    ros::spin();

    if (g_writer.isOpened())
    {
        g_writer.release();
    }

    return 0;
}
