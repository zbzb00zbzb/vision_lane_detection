#include <hadmap_msgs/Map.h>
#include <ros/ros.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <string>

#include "lane_line/Curve.h"
#include "lane_line/Drawing.h"
#include "lane_line/Line.h"

cv::Mat g_image;

void callback(const hadmap_msgs::MapConstPtr &map)
{
    float offset = 500;

    g_image = cv::Mat::zeros(1000, 1000, CV_8UC3);
    cv::circle(g_image, cv::Point(offset, offset), 10,
               cv::Scalar(255, 255, 255), -1);

    auto lanes = map->sections[0].lanes;
    std::cout << "new message received\n";
    std::cout << "there are: " << lanes.size() << " lanes \n";
    for (const auto &lane : lanes)
    {
        float x1, y1, x2, y2;
        // left
        x1 = lane.pts_left[0].point.x;
        y1 = lane.pts_left[0].point.y;

        x2 = lane.pts_left[1].point.x;
        y2 = lane.pts_left[1].point.y;

        std::cout << "lane: " << lane << std::endl;

        x1*= 20;
        y1*= 20;

        x2*= 20;
        y2*= 20;


        x1 += offset;
        y1 += offset;
        x2 += offset;
        y2 += offset;
        tt::Drawing::lineInPlace(g_image, cv::Point(x1, y1),
                                 cv::Point(x2, y2), tt::Drawing::green(), 4);

        // right
        x1 = lane.pts_right[0].point.x;
        y1 = lane.pts_right[0].point.y;

        x2 = lane.pts_right[1].point.x;
        y2 = lane.pts_right[1].point.y;

        x1 += offset;
        y1 += offset;
        x2 += offset;
        y2 += offset;

        tt::Drawing::lineInPlace(g_image, cv::Point(x1, y1),
                                 cv::Point(x2, y2), tt::Drawing::green(), 4);
    }

    cv::namedWindow("image", cv::WINDOW_NORMAL);
    cv::imshow("image", g_image);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "draw_detected_lines");

    std::string topic_name = "/lane_detection/hadmap_new_msg";
    ros::NodeHandle nh("~");
    auto sub = nh.subscribe(topic_name, 10, callback);
    while (ros::ok())
    {
        ros::spinOnce();
        cv::waitKey(10);
    }

    return 0;
}
