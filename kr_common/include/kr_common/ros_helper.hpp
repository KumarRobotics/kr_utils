#ifndef KR_COMMON_ROS_HELPER_HPP_
#define KR_COMMON_ROS_HELPER_HPP_

#include <ros/ros.h>
#include <ros/package.h>

namespace kr {

/**
 * @brief getParam Get ros param in node handle's namespace, with default value
 */
template <typename T>
T GetParam(const ros::NodeHandle& nh, const std::string& param_name,
           const T& default_val) {
  T param_val;
  nh.param<T>(nh.resolveName(param_name), param_val, default_val);
  return param_val;
}

/**
 * @brief GetParam Get ros param in node handle's namespace ,use T's default
 * constructor
 */
template <typename T>
T GetParam(const ros::NodeHandle& nh, const std::string& param_name) {
  const T default_val{};
  return GetParam(nh, param_name, default_val);
}

/**
 * @brief getParam Get ros param in node's namespace with default value
 */
template <typename T>
T GetParam(const std::string& param_name, const T& default_val) {
  T param_val;
  ros::param::param<T>(ros::names::resolve(param_name), param_val, default_val);
  return param_val;
}

template <typename T>
/**
 * @brief GetParam Get ros param in node's namespace, use T's default
 * constructor
 */
T GetParam(const std::string& param_name) {
  const T default_val{};
  return GetParam(param_name, default_val);
}

/**
 * @brief PackageUrlToFullPath Get full file path based on package url
 * @param url Url start with package://
 * @return Path of url
 * @note Assume url contains no special variable needs to be resolved
 */
std::string PackageUrlToFullPath(const std::string url) {
  static const std::string pkg_prefix("package://");
  static const size_t prefix_len = pkg_prefix.length();
  const size_t rest = url.find('/', prefix_len);
  const std::string pkg(url.substr(prefix_len, rest - prefix_len));

  // Look up the ROS package path name.
  const std::string pkg_path(ros::package::getPath(pkg));
  if (pkg_path.empty()) {
    ROS_WARN_STREAM("unknown package: " << pkg << " (ignored)");
    return pkg_path;
  }

  return pkg_path + url.substr(rest);
}

}  // namespace kr

#endif  // KR_COMMON_ROS_HELPER_HPP_
