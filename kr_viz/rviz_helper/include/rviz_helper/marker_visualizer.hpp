#ifndef KR_VIZ_RVIZ_HELPER_MARKER_VISUALIZER_HPP_
#define KR_VIZ_RVIZ_HELPER_MARKER_VISUALIZER_HPP_

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include "rviz_helper/colors.hpp"
#include "rviz_helper/tf_publisher.hpp"

namespace kr {
namespace viz {

using visualization_msgs::Marker;
using geometry_msgs::PoseWithCovariance;
using geometry_msgs::PoseWithCovarianceStamped;

/**
 * @brief The MarkerVisualizer class
 */
class MarkerVisualizer {
 public:
  /**
   * @brief MarkerVisualizer Ctor
   * @param nh ROS node handle
   * @param topic Topic name
   */
  MarkerVisualizer(const ros::NodeHandle &nh, const std::string &topic)
      : nh_(nh), pub_(nh_.advertise<Marker>(topic, 1)) {}

  /**
   * @brief setColor Set marker color
   */
  void SetColor(double r, double g, double b) {
    const rviz::Color color(r, g, b);
    SetColor(color);
  }

  /**
   * @brief setColor set marker color
   */
  void SetColor(const rviz::Color &color) {
    marker_.color.r = color.r_;
    marker_.color.g = color.g_;
    marker_.color.b = color.b_;
  }

  /**
   * @brief setAlpha Set marker alpha
   */
  void SetAlpha(double alpha) { marker_.color.a = alpha; }

  /**
   * @brief setScale
   */
  void SetScale(double s_x, double s_y = 1, double s_z = 1) {
    marker_.scale.x = s_x;
    marker_.scale.y = s_y;
    marker_.scale.z = s_z;
  }
  void SetScale(double *scale) { SetScale(scale[0], scale[1], scale[2]); }

  /**
   * @brief setLifetime Set marker lifetime
   */
  void SetLifetime(const ros::Duration &duration) {
    marker_.lifetime = duration;
  }

  /**
   * @brief SetAction Set marker action
   */
  void SetAction(unsigned int action) { marker_.action = action; }

  /**
   * @brief SetType Set marker type
   */
  void SetType(unsigned int type) { marker_.type = type; }

  void Publish(const Marker &marker) { pub_.publish(marker_); }

 protected:
  Marker marker_;  ///< Visualization marker

 private:
  ros::NodeHandle nh_;  ///< ROS node handle
  ros::Publisher pub_;  ///< ROS publisher
};

class TrajectoryVisualizer : public MarkerVisualizer {
 public:
  /**
   * @brief TrajectoryVisualizer Ctor
   * @param nh ROS node handle
   * @param topic Topic name
   */
  TrajectoryVisualizer(const ros::NodeHandle &nh,
                       const std::string &topic = "traj");

  /**
   * @brief set_num_skip
   * @param num_skip
   */
  void set_num_skip(int num_skip) { num_skip_ = num_skip; }

  /**
   * @brief PublishTrajectory Publish trajectory
   */
  void PublishTrajectory(const geometry_msgs::Point &point,
                         const std_msgs::Header &header);
  void PublishTrajectory(const geometry_msgs::Pose &pose,
                         const std_msgs::Header &header);

 private:
  int num_skip_{1};          ///< Number of points to skip, 1 keeps all
  int total_points_cnt_{0};  ///< Total points in marker
};

/**
 * @brief Visualize covariance as a scaled ellipsoid.
 */
class CovarianceVisualizer : public MarkerVisualizer {
 public:
  CovarianceVisualizer(const ros::NodeHandle &nh,
                       const std::string &topic = "covariance",
                       bool transforms_orientation = false);

  void PublishCovariance(const PoseWithCovariance &pose_cov,
                         const std_msgs::Header &header);
  void PublishCovariance(const PoseWithCovarianceStamped &pose_cov_stamped);
  void PublishCovariance(const nav_msgs::Odometry &odometry);
private:
  bool transform_orientation_{false};
};

/**
 * @brief The PoseVisualizer class
 * This visualizer will also publish a tf
 */
class PoseVisualizer : public TrajectoryVisualizer {
 public:
  PoseVisualizer(const ros::NodeHandle &nh, const std::string &topic = "traj",
                 const std::string &child_frame_id = "")
      : TrajectoryVisualizer(nh, topic), tf_pub_(child_frame_id) {}

  const std::string &child_frame_id() const { return tf_pub_.child_frame_id(); }
  void set_child_frame_id(const std::string &frame_id) {
    tf_pub_.set_child_frame_id(frame_id);
  }

  void PublishPose(const geometry_msgs::Pose &pose,
                   const std_msgs::Header &header);

 private:
  TfPublisher tf_pub_;
};

}  // namespace viz
}  // namespace kr

#endif  // KR_VIZ_RVIZ_HELPER_MARKER_VISUALIZER_HPP_
