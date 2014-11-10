#include "kr_common/ros_helper.hpp"
#include <gtest/gtest.h>

using namespace kr;
using namespace std;

class RosUrlParserTest : public testing::Test {
 protected:
  RosUrlParserTest()
      : file_name("test.yaml"),
        url_file(string("file:///") + getenv("HOME") + string("/.ros/") +
                 file_name),
        url_file_good(string("file:///${ROS_HOME}/") + file_name),
        url_file_env_bad(string("file:///${ROS_HME}/") + file_name),
        url_file_header_bad(string("fle:///${ROS_HME}/") + file_name) {}

 protected:
  string file_name;
  string url_file, url_file_good, url_file_env_bad, url_file_header_bad;
};

TEST_F(RosUrlParserTest, ResolveUrl) {
  EXPECT_EQ(RosUrlParser::ResolveUrl(url_file_good), url_file);
  EXPECT_NE(RosUrlParser::ResolveUrl(url_file_env_bad), url_file);
  EXPECT_NE(RosUrlParser::ResolveUrl(url_file_header_bad), url_file);
}

TEST_F(RosUrlParserTest, ValidateUrl) {
  EXPECT_TRUE(RosUrlParser::ValidateUrl(url_file_good));
  EXPECT_FALSE(RosUrlParser::ValidateUrl(url_file_header_bad));
}
