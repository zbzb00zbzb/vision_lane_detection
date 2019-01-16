/**
 * @file raw_image_test.cpp
 * @author Kuang Fangjun <csukuangfj@gmail.com>
 * @date July 31, 2018
 * @brief Take the raw image as input for segnet.
 */

#include <string>

#include "lane_line/config.h"
#include "lane_line/DateTime.h"
#include "lane_line/FileSystem.h"
#include "lane_line/LaneLineDetector.h"

int main(int argc, char * argv[])
{
    FLAGS_alsologtostderr = 1;
    FLAGS_colorlogtostderr = 1;
    google::InitGoogleLogging(argv[0]);

     //==============================
     //  Load config
     //------------------------------
    std::string config_filename = CONFIG_PROTO_FILENAME;

    tt::LaneLineDetectorProto param;
    tt::loadConfigFromFile(config_filename, &param);
    tt::checkConfig(param);

    LOG(INFO) << param.DebugString();

    std::string filename = std::string("./current-config-") +
                           tt::DateTime::toString()+ ".txt";
    tt::saveConfigToFile(filename, param);
    tt::saveConfigToFile("./latest-config.txt", param);

    auto detector = tt::LaneLineDetector::create(param);
    CHECK(detector) << "Failed to create lane line detector for "
                    << param.type();

    std::string dir = "/home/fangjun/Desktop/images/done/2018-07-12-16-55-21";

    auto list_of_images = tt::FileSystem::getListOfFiles(dir, {"jpg"});

    LOG(INFO) << "There are " << list_of_images.size() << " images";

    for (size_t i = 187; i < list_of_images.size();)
    {
        LOG(INFO) << i << "/" << list_of_images.size();
        auto image = cv::imread(list_of_images[i], cv::IMREAD_COLOR);
        cv::resize(image, image, cv::Size(960, 640));

        auto sub = image(cv::Rect(100, 40, 600, 600));
        cv::resize(sub, sub, cv::Size(448, 448));

        detector->preprocessing()->doPreprocessing(sub);

        auto prob = detector->preprocessing()->getResult();

        cv::imshow("image", image);
        cv::imshow("sub", sub);
        cv::imshow("prob", prob);
        int key = cv::waitKey(0);
        if (key == 'q') break;
        else if (key == 'n') i++;
        else if (key == 'p') i--;
    }

    return 0;
}



