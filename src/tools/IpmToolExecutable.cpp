/**
 * @file IPMToolExecutable.cpp
 * @author Kuang Fangjun <csukuangfj@gmail.com>
 * @date June 05, 2018
 * @brief IPM Calibration.
 */

#include <algorithm>
#include <sstream>

#include "lane_line/common.h"

#include "lane_line/DateTime.h"
#include "lane_line/FileSystem.h"
#include "lane_line/IPMParameters.h"
#include "lane_line/IPMTransformation.h"

namespace
{


enum class PLAY_MODE {
    STEP,
    CONTINUOUS,
};

enum class STATUS {
    play = 'm',
    fx = '1',
    fy = '2',
    cx = '3',
    cy = '4',
    x1 = '5',
    y1 = '6',
    x2 = '7',
    y2 = '8',
    pitch = 'p',
    yaw = 'y',
    roll = 'r',
    step_size = 's',
    increase = 'i',
    decrease = 'd',

    save = '0',    //!< save parameters to file
};


}  // end namespace

static void parseCommandline(int argc, char *argv[], tt::IPMParameters &params); // NOLINT
static void handleKeyboard(int key);
static void showStatus();
static void process(STATUS key);
static void updateTransform();
static void saveParamToFile();

tt::IPMTransformation g_tf;

PLAY_MODE g_play_mode = PLAY_MODE::STEP;
STATUS g_status = STATUS::play;
tt::IPMParameters g_params;
float g_step_size = 0.1;
cv::String g_image_dir;
cv::String g_ipm_result_dir;

int
main(int argc, char *argv[])
{
    FLAGS_alsologtostderr = 1;
    FLAGS_colorlogtostderr = 1;
    google::InitGoogleLogging(argv[0]);

    parseCommandline(argc, argv, g_params);
    LOG(INFO) << g_params.toString() << "\n";
    showStatus();
    updateTransform();

    cv::Mat raw, ipm;

    auto list_of_images = tt::FileSystem::getListOfFiles(g_image_dir, {".jpg"});

    // cv::namedWindow("raw", 0);
    for (size_t i = 188; i < list_of_images.size(); i++)
    {
        LOG(INFO) << "read " << i << "/" << list_of_images.size() << ": "
                  << list_of_images[i];

        raw = cv::imread(list_of_images[i], cv::IMREAD_COLOR);
        cv::resize(raw, raw, cv::Size(960, 600));
        if (raw.empty()) continue;

        ipm = g_tf.computeIPMImage(raw);

        cv::imshow("raw", raw);
        cv::imshow("ipm", ipm);

        int key;
        if (g_play_mode == PLAY_MODE::STEP)
        {
            key = cv::waitKey(0);
        }
        else  // NOLINT
        {
            key = cv::waitKey(20);
        }
        if (key == 'q') break;

        handleKeyboard(key);
        if (key != ' ' && g_play_mode != PLAY_MODE::CONTINUOUS) i--;
    }
    return 0;
}

static
void
parseCommandline(int argc, char *argv[], tt::IPMParameters &params)  // NOLINT
{
    cv::String keys = {
            "{help h usage ? |     | show help}"
                    "{pitch |     | pitch}"
                    "{yaw   |     | yaw}"
                    "{roll  |     | roll}"
                    "{cx    |     | cx}"
                    "{cy    |     | cy}"
                    "{fx    |     | fx}"
                    "{fy    |     | fy}"
                    "{x1    |     | x1}"
                    "{y1    |     | y1}"
                    "{x2    |     | x2}"
                    "{y2    |     | y2}"
                    "{out_width    |     | out_width}"
                    "{out_height   |     | out_height}"
                    "{image_dir    |     | image directory}"
                    "{ipm_result_dir| | ipm result directory}"
    };

    cv::CommandLineParser parser(argc, argv, keys);
    if (parser.has("help"))
    {
        parser.printMessage();
        exit(0);
    }
#define MY_READ(p, type) \
    if (!parser.has(#p)) \
    {\
        LOG(FATAL) << "Cannot read " << #p << "\n";\
        exit(-1); \
    }\
    params. p = parser.get<type>(#p);

    MY_READ(pitch, float);
    MY_READ(yaw, float);
    MY_READ(roll, float);
    MY_READ(cx, float);
    MY_READ(cy, float);
    MY_READ(fx, float);
    MY_READ(fy, float);
#undef MY_READ

    if (!parser.has("x1")) LOG(FATAL) << "Cannot read " << "x1" << "\n";
    params.roi_in.x = parser.get<int>("x1");

    if (!parser.has("y1")) LOG(FATAL) << "Cannot read " << "x1" << "\n";
    params.roi_in.y = parser.get<int>("y1");

    if (!parser.has("x2")) LOG(FATAL) << "Cannot read " << "x2" << "\n";
    params.roi_in.width = parser.get<int>("x2") - params.roi_in.x;

    if (!parser.has("y2")) LOG(FATAL) << "Cannot read " << "y2" << "\n";
    params.roi_in.height = parser.get<int>("y2") - params.roi_in.y;

    if (!parser.has("out_width")) LOG(FATAL) << "Cannot read " << "out_width";
    params.roi_out.width = parser.get<int>("out_width");

    if (!parser.has("out_height")) LOG(FATAL) << "Cannot read " << "out_height";
    params.roi_out.height = parser.get<int>("out_height");

    if (!parser.has("image_dir")) LOG(FATAL) << "Cannot read " << "image_dir";
    g_image_dir = parser.get<cv::String>("image_dir");

    if (!parser.has("ipm_result_dir")) LOG(FATAL) << "Cannot read " << "ipm_result_dir";    // NOLINT
    g_ipm_result_dir = parser.get<cv::String>("ipm_result_dir");

    LOG(INFO) << "ipm_result_dir: " << g_ipm_result_dir;
}

static
void
handleKeyboard(int key)
{
    switch (key)
    {
        case static_cast<int>(STATUS::play):
            g_status = STATUS::play;
            break;
        case static_cast<int>(STATUS::fx):
            g_status = STATUS::fx;
            break;
        case static_cast<int>(STATUS::fy):
            g_status = STATUS::fy;
            break;
        case static_cast<int>(STATUS::cx):
            g_status = STATUS::cx;
            break;
        case static_cast<int>(STATUS::cy):
            g_status = STATUS::cy;
            break;
        case static_cast<int>(STATUS::x1):
            g_status = STATUS::x1;
            break;
        case static_cast<int>(STATUS::y1):
            g_status = STATUS::y1;
            break;
        case static_cast<int>(STATUS::x2):
            g_status = STATUS::x2;
            break;
        case static_cast<int>(STATUS::y2):
            g_status = STATUS::y2;
            break;
        case static_cast<int>(STATUS::pitch):
            g_status = STATUS::pitch;
            break;
        case static_cast<int>(STATUS::yaw):
            g_status = STATUS::yaw;
            break;
        case static_cast<int>(STATUS::roll):
            g_status = STATUS::roll;
            break;
        case static_cast<int>(STATUS::step_size):
            g_status = STATUS::step_size;
            break;
        case static_cast<int>(STATUS::increase):
            process(STATUS::increase);
            break;
        case static_cast<int>(STATUS::decrease):
            process(STATUS::decrease);
            break;
        case static_cast<int>(STATUS::save):
            saveParamToFile();
            break;
        default:
            LOG(INFO) << "unknown key: " << static_cast<char>(key) << "\n";
            LOG(INFO) << "ignore it\n";
            break;
    }
    showStatus();
}


static
void
showStatus()
{
    std::stringstream ss;
    ss << "\n\n\n";
    ss << "\n--------------------\n";
    if (g_status == STATUS::play)
        ss << "(m). play mode: " << ((g_play_mode == PLAY_MODE::STEP) ?
                                     "step" : "continuous") << "\n";
    else
        ss << "m. play mode: " << ((g_play_mode == PLAY_MODE::STEP) ?
                                   "step" : "continuous") << "\n";

    if (g_status == STATUS::fx)
        ss << "(1). fx: " << g_params.fx << "\n";
    else
        ss << "1. fx: " << g_params.fx << "\n";

    if (g_status == STATUS::fy)
        ss << "(2). fy: " << g_params.fy << "\n";
    else
        ss << "2. fy: " << g_params.fy << "\n";

    if (g_status == STATUS::cx)
        ss << "(3). cx: " << g_params.cx << "\n";
    else
        ss << "3. cx: " << g_params.cx << "\n";

    if (g_status == STATUS::cy)
        ss << "(4). cy: " << g_params.cy << "\n";
    else
        ss << "4. cy: " << g_params.cy << "\n";

    if (g_status == STATUS::x1)
        ss << "(5). x1: " << g_params.roi_in.x << "\n";
    else
        ss << "5. x1: " << g_params.roi_in.x << "\n";

    if (g_status == STATUS::y1)
        ss << "(6). y1: " << g_params.roi_in.y << "\n";
    else
        ss << "6. y1: " << g_params.roi_in.y << "\n";

    if (g_status == STATUS::x2)
        ss << "(7). x2: " << g_params.roi_in.br().x << "\n";
    else
        ss << "7. x2: " << g_params.roi_in.br().x << "\n";

    if (g_status == STATUS::y2)
        ss << "(8). y2: " << g_params.roi_in.br().y << "\n";
    else
        ss << "8. y2: " << g_params.roi_in.br().y << "\n";

    if (g_status == STATUS::pitch)
        ss << "(p). pitch: " << g_params.pitch << "\n";
    else
        ss << "p. pitch: " << g_params.pitch << "\n";

    if (g_status == STATUS::yaw)
        ss << "(y). yaw: " << g_params.yaw << "\n";
    else
        ss << "y. yaw: " << g_params.yaw << "\n";

    if (g_status == STATUS::roll)
        ss << "(r). roll: " << g_params.roll << "\n";
    else
        ss << "r. roll: " << g_params.roll << "\n";

    if (g_status == STATUS::step_size)
        ss << "(s). step size: " << g_step_size << "\n";
    else
        ss << "s. step size: " << g_step_size << "\n";

    ss << "i: increase\n";
    ss << "d: decrease\n";
    ss << "out_width: " << g_params.roi_out.width << "\n";
    ss << "out_height: " << g_params.roi_out.height << "\n";
    ss << "0: save parameters to file\n";
    ss << "\n--------------------\n";
    LOG(INFO) << ss.str();
}

static
void
process(STATUS key)
{
    int sign = 1;
    if (key == STATUS::decrease) sign = -1;
    switch (g_status)
    {
        case STATUS::play:
            g_play_mode = (g_play_mode == PLAY_MODE::STEP) ?
                          PLAY_MODE::CONTINUOUS : PLAY_MODE::STEP;
            break;
        case STATUS::fx:
            g_params.fx += sign * g_step_size;
            g_params.fx = cv::max(g_params.fx, 20.0f);
            break;
        case STATUS::fy:
            g_params.fy += sign * g_step_size;
            g_params.fy = cv::max(g_params.fy, 20.0f);
            break;
        case STATUS::cx:
            g_params.cx += sign * g_step_size;
            g_params.cx = cv::max(g_params.cx, 20.0f);
            break;
        case STATUS::cy:
            g_params.cy += sign * g_step_size;
            g_params.cy = cv::max(g_params.cy, 20.0f);
            break;
        case STATUS::x1:
            g_params.roi_in.x += sign;
            g_params.roi_in.width -= sign;
            g_params.roi_in.x = cv::min(g_params.roi_in.br().x,
                                        g_params.roi_in.x);
            g_params.roi_in.x = cv::max(0, g_params.roi_in.x);
            break;
        case STATUS::y1:
            g_params.roi_in.y += sign;
            g_params.roi_in.height -= sign;
            g_params.roi_in.y = cv::min(g_params.roi_in.br().y-1,
                                        g_params.roi_in.y);
            g_params.roi_in.y = cv::max(0, g_params.roi_in.y);
            break;
        case STATUS::x2:
            g_params.roi_in.width += sign;
            g_params.roi_in.width = cv::max(1, g_params.roi_in.width);
            break;
        case STATUS::y2:
            g_params.roi_in.height += sign;
            g_params.roi_in.height = cv::max(1, g_params.roi_in.height);
            break;
        case STATUS::pitch:
            g_params.pitch += sign * g_step_size;
            break;
        case STATUS::yaw:
            g_params.yaw += sign * g_step_size;
            break;
        case STATUS::roll:
            g_params.roll += sign * g_step_size;
            break;
        case STATUS::step_size:
            g_step_size += sign * 0.1;
            break;
    }
    updateTransform();
}

static
void
updateTransform()
{
    g_tf.init(g_params);
}

static
void
saveParamToFile()
{
    auto filename = tt::DateTime::toString();
    filename = g_ipm_result_dir + "/" + filename + ".txt";

    std::ostringstream ss;
    ss << "# " << g_image_dir << "\n";
    ss << g_params.toString();

    auto ret = tt::FileSystem::saveToTextFile(filename,
                                              ss.str());
    if (!ret)
        LOG(WARNING) << "Failed to save parameters to file!";
    else
        LOG(WARNING) << "Saved parameters to file " << filename;
}

