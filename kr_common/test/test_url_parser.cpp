#include <kr_common/ros_helper.hpp>

using namespace kr;

int main(int argc, char** argv) {
  // Good url
  const std::string url1 = "file://${ROS_HOME}/test.yaml";
  const std::string url2 = "package://kr_common/test.yaml";
  const std::string url3 = "file:///home/chao/.ros/test.yaml";

  // Bad url
  const std::string url4 = "file://${ROS_HME}/test.yaml";
  const std::string url5 = "packge://kr_common/test.yaml";
  const std::string url6 = "package://kr_comon/test.yaml";
  const std::string url7 = "fle://kr_comon/test.yaml";

  std::cout << "+++ Good url" << std::endl;
  ROS_INFO_STREAM(RosUrlParser::UrlToFullPath(url1));
  ROS_INFO_STREAM(RosUrlParser::UrlToFullPath(url2));
  ROS_INFO_STREAM(RosUrlParser::UrlToFullPath(url3));

  std::cout << "\n--- Bad url" << std::endl;
  ROS_INFO_STREAM(RosUrlParser::UrlToFullPath(url4));
  ROS_INFO_STREAM(RosUrlParser::UrlToFullPath(url5));
  ROS_INFO_STREAM(RosUrlParser::UrlToFullPath(url6));
  ROS_INFO_STREAM(RosUrlParser::UrlToFullPath(url7));
}
