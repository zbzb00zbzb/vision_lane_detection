/**
 * @file FileSystem.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 04, 2018
 */

#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <algorithm>
#include <fstream>  // NOLINT
#include <string>
#include <vector>

#include "lane_line/common.h"
#include "lane_line/FileSystem.h"

namespace
{

bool endsWidth(const std::string &str,
               const std::vector<std::string> &ext)
{
    bool res = false;
    for (const auto &s : ext)
    {
        auto pos = str.rfind(s);
        if (pos == std::string::npos) continue;

        if (s == str.substr(pos))
        {
            res = true;
            break;
        }
    }

    return res;
}

}  // namespace

namespace tt
{


bool FileSystem::fileExists(const std::string &filename)
{
    bool res = false;
    struct stat sb;
    if (!stat(filename.c_str(), &sb))
    {
        res = S_ISREG(sb.st_mode);
    }

    return res;
}

bool FileSystem::directoryExists(const std::string &directory_name)
{
    bool res = false;
    struct stat sb;
    if (!stat(directory_name.c_str(), &sb))
    {
        res = S_ISDIR(sb.st_mode);
    }

    return res;
}

bool FileSystem::createDirectory(const std::string &directory_name)
{
    if (directoryExists(directory_name)) return true;

    char buf[500] = {0};
    snprintf(buf, sizeof(buf), "mkdir -p %s", directory_name.c_str());

    int res = system(buf);
    return res == 0;
}

bool FileSystem::saveToTextFile(const std::string &filename,
                                const std::string &content)
{
    std::ofstream f;
    f.open(filename, std::ios::trunc);
    if (!f.is_open())
    {
        LOG(ERROR) << "Failed to create file " << filename;
        return false;
    }
    f << content;
    f.close();

    return true;
}

std::vector<std::string> FileSystem::getListOfFiles(
        const std::string &abs_dir,
        const std::vector<std::string> &ext)
{
    std::vector<std::string> res;
    DIR *dir = opendir(abs_dir.c_str());
    if (dir)
    {
        struct dirent *ent;
        while ((ent = readdir(dir)))
        {
            if (ent->d_type != DT_REG) continue;

            std::string s{ent->d_name};
            if (endsWidth(s, ext))
            {
                res.push_back(abs_dir + "/" + s);
            }
        }

        closedir(dir);
    }

    std::sort(res.begin(), res.end());
    return res;
}


}  // namespace tt
