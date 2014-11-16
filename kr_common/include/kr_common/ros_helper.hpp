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
  return GetParam<T>(nh, param_name, default_val);
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
  return GetParam<T>(param_name, default_val);
}

/**
 * @brief PackageUrlToFullPath Get full file path based on package url
 * @param url Url start with package://
 * @return Path of url
 * @note Assume url contains no special variable needs to be resolved
 */
static inline std::string PackageUrlToFullPath(const std::string url) {
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

/**
 * @brief The UrlParser class
 */
class RosUrlParser {
 public:
  /**
   * @brief UrlToFullPath Get full path based on url
   * @param url Url start with package:// or file://
   * @return Path of url if valid, empty string if invalid
   */
  static std::string UrlToFullPath(const std::string& url) {
    const std::string url_resolved(ResolveUrl(url));
    const auto url_type = ParseUrl(url_resolved);
    if (url_type == UrlType::URL_FILE) {
      return url_resolved.substr(7);
    }
    if (url_type == UrlType::URL_PACKAGE) {
      return PackageUrlToFullPath(url_resolved);
    }
    return std::string{};
  }

  /**
   * @brief ResolveUrl Resolve variables in url
   * @return Resolved url
   */
  static std::string ResolveUrl(const std::string& url) {
    std::string resolved;
    size_t rest = 0;

    while (true) {
      // find the next '$' in the URL string
      auto dollar = url.find('$', rest);

      if (dollar >= url.length()) {
        // no more variables left in the URL
        resolved += url.substr(rest);
        break;
      }

      // copy characters up to the next '$'
      resolved += url.substr(rest, dollar - rest);

      if (url.substr(dollar + 1, 1) != "{") {
        // no '{' follows, so keep the '$'
        resolved += "$";
      } else if (url.substr(dollar + 1, 10) == "{ROS_HOME}") {
        // substitute $ROS_HOME
        std::string ros_home;
        char* ros_home_env;
        if ((ros_home_env = getenv("ROS_HOME"))) {
          // use environment variable
          ros_home = ros_home_env;
        } else if ((ros_home_env = getenv("HOME"))) {
          // use "$HOME/.ros"
          ros_home = ros_home_env;
          ros_home += "/.ros";
        }
        resolved += ros_home;
        dollar += 10;
      } else {
        // not a valid substitution variable
        ROS_ERROR_STREAM(
            "[UrlParser]"
            " invalid URL substitution (not resolved): "
            << url);
        resolved += "$";  // keep the bogus '$'
      }

      // look for next '$'
      rest = dollar + 1;
    }

    return resolved;
  }

  /**
   * @brief ValidateUrl Validate if url is supported
   * @return True if url is valid
   */
  static bool ValidateUrl(const std::string& url) {
    UrlType url_type = ParseUrl(ResolveUrl(url));
    return (url_type < UrlType::URL_INVALID);
  }

 private:
  enum class UrlType {
    URL_EMPTY = 0,  // empty string
    URL_FILE,       // file:
    URL_PACKAGE,    // package:
    URL_INVALID     // anything >= is invalid
  };

  /**
   * @brief ParseUrl Parse resolved url
   * @return Url type
   */
  static UrlType ParseUrl(const std::string& url_resolved) {
    if (url_resolved == "") {
      return UrlType::URL_EMPTY;
    }
    if (url_resolved.substr(0, 8) == "file:///") {
      return UrlType::URL_FILE;
    }
    if (url_resolved.substr(0, 10) == "package://") {
      auto rest = url_resolved.find('/', 10);
      if (rest < url_resolved.length() - 1 && rest > 10) {
        return UrlType::URL_PACKAGE;
      }
    }
    return UrlType::URL_INVALID;
  }
};

}  // namespace kr

#endif  // KR_COMMON_ROS_HELPER_HPP_
