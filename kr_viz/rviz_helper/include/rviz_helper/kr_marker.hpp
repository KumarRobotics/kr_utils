#ifndef KR_MARKER_HPP_
#define KR_MARKER_HPP_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <rviz_helper/colors.hpp>
#include <Eigen/Core>
#include <geometry_msgs/PoseWithCovariance.h>


namespace kr {
namespace viz {

// Defines visualization marker with better default syntax and coloring

class Marker {
 public:
  Marker();
  Marker(const visualization_msgs::Marker &mark) { mark_ = mark; }
  operator visualization_msgs::Marker() { return mark_; }
  operator visualization_msgs::MarkerArray();

  // set marker properties using chained structure:
  // ex: mahkah.color(kr::viz::colors::Pink).frame_id("sim")
  Marker &color(const rviz::Color &col);
  Marker &color(const std_msgs::ColorRGBA &col);
  Marker &scale(const geometry_msgs::Vector3 &sca);
  Marker &scale(double x, double y, double z);
  Marker &scale(double s);
  Marker &mesh(const std::string &resource);
  Marker &type(int32_t tp);
  Marker &point_push_back(const geometry_msgs::Point &pt);
  Marker &position(const geometry_msgs::Point pose);
  Marker &position(double x, double y, double z);
  Marker &orientation(double dx, double dy, double dz);
  Marker &alpha(double a);
  Marker &action(u_int8_t a);
  Marker &covariance(const Eigen::Matrix3d &cov);
  Marker &covariance(const geometry_msgs::PoseWithCovariance &pose_cov);

 private:
  int getUniqueId();

 private:
  visualization_msgs::Marker mark_;
};  // class Marker

class MarkerArray {
 public:
  MarkerArray();
  MarkerArray(const visualization_msgs::MarkerArray &array);
  // hollow rectangle
  MarkerArray(double lx, double ly, double lz, double ux, double uy, double uz,
              rviz::Color col);

  MarkerArray operator+(const MarkerArray &array);
  MarkerArray operator+(const Marker &marker);
  MarkerArray operator+=(const MarkerArray &array);
  MarkerArray operator+=(const Marker &marker);

  operator visualization_msgs::MarkerArray();

  void push_back(const Marker &mark);
  void push_back(const MarkerArray &mark);

  Marker &at(int pos);
  const Marker &at(int pos) const;
  size_t size() const { return array_.size(); }

  //batch marker operation

  MarkerArray &color(const rviz::Color &col);
  MarkerArray &color(const std_msgs::ColorRGBA &col);
  MarkerArray &scale(const geometry_msgs::Vector3 &sca);
  MarkerArray &scale(double x, double y, double z);
  MarkerArray &scale(double s);
  MarkerArray &mesh(const std::string &resource);
  MarkerArray &type(int32_t tp);
  MarkerArray &point_push_back(const geometry_msgs::Point &pt);
  MarkerArray &position(const geometry_msgs::Point pose);
  MarkerArray &position(double x, double y, double z);
  MarkerArray &alpha(double a);
  MarkerArray &action(u_int8_t a);

 private:
  std::vector<Marker> array_;
};

}  // namespace viz
}  // namespace kr

#endif  // KR_MARKER_HPP
