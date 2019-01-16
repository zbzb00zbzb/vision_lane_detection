/**
 * @file FileSystem.h
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 04, 2018
 */

#ifndef LANE_LINE_FILESYSTEM_H
#define LANE_LINE_FILESYSTEM_H

#include <string>
#include <vector>

namespace tt
{

class FileSystem
{
 public:
    /**
     *
     * @param filename  file name to be checked.
     * @return True if the file is a regular file, false otherwise.
     */
    static bool fileExists(const std::string &filename);
    static bool directoryExists(const std::string &directory_name);

    /**
     * @param directory_name the directory name to be created.
     * @return True if the directory is created successfully, false otherwise.
     */
    static bool createDirectory(const std::string &directory_name);

    /**
     * Save content to a text file. If the file exists, it is overwritten.
     *
     * @param filename file name.
     * @param content Text string to be written.
     * @return true on success, false otherwise.
     */
    static bool saveToTextFile(const std::string &filename,
                               const std::string &content);

    static std::vector<std::string> getListOfFiles(
            const std::string &abs_dir,
            const std::vector<std::string> &ext);
};


}  // namespace tt

#endif  // LANE_LINE_FILESYSTEM_H
