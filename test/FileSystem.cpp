/**
 * @file FileSystem.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 04, 2018
 */

#include <gtest/gtest.h>
#include <string>

#include "lane_line/DateTime.h"
#include "lane_line/FileSystem.h"


class FileSystemTest : public ::testing::Test
{
 protected:
    tt::FileSystem fs_;
};

TEST_F(FileSystemTest, file_exists)
{
    std::string filename = "/etc/passwd";

    auto res = fs_.fileExists(filename);
    EXPECT_TRUE(res);

    filename = "/etc/passwort";
    res = fs_.fileExists(filename);
    EXPECT_FALSE(res);
}

TEST_F(FileSystemTest, directory_exists)
{
    std::string directory_name = "/etc";
    auto res = fs_.directoryExists(directory_name);
    EXPECT_TRUE(res);

    directory_name = "/not-exist";
    res = fs_.directoryExists(directory_name);
    EXPECT_FALSE(res);
}

TEST_F(FileSystemTest, create_directory)
{
    std::string directory_name = "/tmp/abc";
    auto res = fs_.createDirectory(directory_name);
    EXPECT_TRUE(res);

    directory_name = "/root/abc";
    res = fs_.createDirectory(directory_name);
    EXPECT_FALSE(res);
}

TEST_F(FileSystemTest, write_text_to_file)
{
    auto filename = tt::DateTime::toString();
    filename += ".txt";

    std::stringstream ss;
    ss << "# " << filename << "\n";
    ss << "fx: " << 13 << "\n";
    ss << "fy: " << 20 << "\n";

    auto ret = fs_.saveToTextFile(filename, ss.str());
    EXPECT_TRUE(ret);
}
