#ifndef KR_COMMON_ROS_HELPER_HPP_
#define KR_COMMON_ROS_HELPER_HPP_

#include <ros/ros.h>
#include <ros/package.h>

namespace kr {
namespace common {

/**
 * @brief getParam Get ros param in node handle's namespace
 */
template <typename T>
T GetParam(const ros::NodeHandle& nh, const std::string& param_name,
           const T& default_val) {
  T param_val;
  nh.param<T>(nh.resolveName(param_name), param_val, default_val);
  return param_val;
}

template <typename T>
T GetParam(const ros::NodeHandle& nh, const std::string& param_name) {
  T default_val{};
  return GetParam(nh, param_name, default_val);
}

/**
 * @brief getParam Get ros param in node's namespace
 */
template <typename T>
T GetParam(const std::string& param_name, const T& default_val) {
  T param_val;
  ros::param::param<T>(ros::names::resolve(param_name), param_val, default_val);
  return param_val;
}

template <typename T>
T GetParam(const std::string& param_name) {
  T default_val{};
  return GetParam(param_name, default_val);
}

/**
 * @brief GetPackageFilename Get full file name based on url
 * @param url Url start with package://
 * @return Path of url
 */
std::string GetPackageFilename(const std::string url) {
  static const std::string pkg_prefix("package://");
  static const size_t prefix_len = pkg_prefix.length();
  size_t rest = url.find('/', prefix_len);
  std::string pkg(url.substr(prefix_len, rest - prefix_len));

  // Look up the ROS package path name.
  std::string pkg_path(ros::package::getPath(pkg));
  if (pkg_path.empty()) {
    ROS_WARN_STREAM("unknown package: " << pkg << " (ignored)");
    return pkg_path;
  } else {
    return pkg_path + url.substr(rest);
  }
}

}  // namespace common
}  // namespace kr

#endif  // KR_COMMON_ROS_HELPER_HPP_
