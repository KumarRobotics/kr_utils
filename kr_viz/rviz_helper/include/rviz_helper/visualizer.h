#ifndef RVIZ_HELPER_VISUALIZER_H_
#define RVIZ_HELPER_VISUALIZER_H_

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_broadcaster.h>
#include <rviz_helper/colors.h>

namespace kr {
namespace rviz_helper {

class TrajectoryVisualizer {
 public:
  TrajectoryVisualizer(const ros::NodeHandle &nh,
                       const std::string &topic = std::string("traj"));

  void set_color(const rviz::Color &color) {
    marker_.color.r = color.r_;
    marker_.color.g = color.g_;
    marker_.color.b = color.b_;
  }
  void set_alpha(double alpha) { marker_.color.a = alpha; }
  void set_scale(double scale) { marker_.scale.x = scale; }
  void set_lifetime(const ros::Duration &duration) {
    marker_.lifetime = duration;
  }
  void set_num_skip(int num_skip) { num_skip_ = num_skip; }

  void PublishTrajectory(const geometry_msgs::Pose &pose,
                         const std_msgs::Header &header);
  void PublishTrajectory(const geometry_msgs::Point &point,
                         const std_msgs::Header &header);
  void PublishTrajectory(const geometry_msgs::Point &point,
                         const std::string &frame_id, const ros::Time &stamp);

 private:
  ros::NodeHandle nh_;
  ros::Publisher traj_pub_;
  int num_skip_;
  int total_points_cnt_;
  visualization_msgs::Marker marker_;
};

/**
 * @brief Visualize covariance as a scaled ellipsoid.
 */
class CovarianceVisualizer {
 public:
  CovarianceVisualizer(const ros::NodeHandle &nh,
                       const std::string &topic = std::string("covariance"));

  void set_color(const rviz::Color &color) {
    marker_.color.r = color.r_;
    marker_.color.g = color.g_;
    marker_.color.b = color.b_;
  }
  void set_alpha(double alpha) { marker_.color.a = alpha; }

  void PublishCovariance(const geometry_msgs::PoseWithCovariance &pose_cov,
                         const std_msgs::Header &header);
  void PublishCovariance(
      const geometry_msgs::PoseWithCovarianceStamped &pose_cov_stamped);
  void PublishCovariance(const nav_msgs::Odometry &odometry);

 private:
  ros::NodeHandle nh_;
  ros::Publisher cov_pub_;
  visualization_msgs::Marker marker_;
};

class TfPublisher {
 public:
  TfPublisher() = default;
  TfPublisher(const std::string &child_frame_id)
      : child_frame_id_(child_frame_id) {}

  const std::string &child_frame_id() const { return child_frame_id_; }
  void set_child_frame_id(const std::string &id) { child_frame_id_ = id; }

  void PublishTransform(const geometry_msgs::Pose &pose,
                        const std_msgs::Header &header);
  void PublishTransform(const geometry_msgs::Pose &pose,
                        const std::string &frame_id, const ros::Time &stamp);

 private:
  std::string child_frame_id_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
};

class PoseVisualizer {
 public:
  PoseVisualizer(const ros::NodeHandle &nh, const std::string &topic = "traj",
                 const std::string &child_frame_id = "")
      : nh_(nh), traj_viz_(nh, topic), tf_pub_(child_frame_id) {}

  void set_color(const rviz::Color &color) { traj_viz_.set_color(color); }
  void set_alpha(double alpha) { traj_viz_.set_alpha(alpha); }
  void set_scale(double scale) { traj_viz_.set_scale(scale); }

  const std::string &child_frame_id() const { return tf_pub_.child_frame_id(); }
  void set_child_frame_id(const std::string &frame_id) {
    tf_pub_.set_child_frame_id(frame_id);
  }

  void PublishPose(const geometry_msgs::Pose &pose,
                   const std_msgs::Header &header);
  void PublishPose(const geometry_msgs::Pose &pose, const std::string &frame_id,
                   const ros::Time &stamp);

 private:
  ros::NodeHandle nh_;
  TrajectoryVisualizer traj_viz_;
  TfPublisher tf_pub_;
};

}  // namespace rviz_helper
}  // namespace kr

#endif  // RVIZ_HELPER_VISUALIZER_H_
