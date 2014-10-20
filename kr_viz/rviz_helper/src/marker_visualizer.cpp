#include "rviz_helper/marker_visualizer.hpp"

namespace kr {
namespace viz {

TrajectoryVisualizer::TrajectoryVisualizer(const ros::NodeHandle &nh,
                                           const std::string &topic)
    : MarkerVisualizer(nh, topic) {
  marker_.pose.orientation.w = 1.0;
  SetColor(colors::RED);
  SetScale(0.05);
  SetLifetime(ros::Duration());
  SetAction(Marker::ADD);
  SetType(Marker::LINE_STRIP);
}

void TrajectoryVisualizer::PublishTrajectory(const geometry_msgs::Point &point,
                                             const std_msgs::Header &header) {
  ++total_points_cnt_;
  if (total_points_cnt_ % num_skip_) return;
  marker_.points.push_back(point);
  marker_.header = header;
  Publish(marker_);
}

void TrajectoryVisualizer::PublishTrajectory(const geometry_msgs::Pose &pose,
                                             const std_msgs::Header &header) {
  PublishTrajectory(pose.position, header);
}

CovarianceVisualizer::CovarianceVisualizer(const ros::NodeHandle &nh,
                                           const std::string &topic)
    : MarkerVisualizer(nh, topic) {
  //  default to a beige colour
  SetColor(1, 0.9255, 0.5);
  SetAction(Marker::ADD);
  SetType(Marker::SPHERE);
  SetLifetime(ros::Duration());  // last forever
}

void CovarianceVisualizer::PublishCovariance(const PoseWithCovariance &pose_cov,
                                             const std_msgs::Header &header) {
  marker_.header = header;
  marker_.pose.position = pose_cov.pose.position;
  marker_.pose.orientation.w = 1;

  double scale[3] = {0, 0, 0};
  for (int i = 0; i < 3; i++) {
    if (pose_cov.covariance[(i * 6) + i] >= 0) {
      scale[i] = std::sqrt(pose_cov.covariance[(i * 6) + i]);
    }
  }

  SetScale(scale);
  Publish(marker_);
}

void CovarianceVisualizer::PublishCovariance(
    const PoseWithCovarianceStamped &pose_cov_stamped) {
  PublishCovariance(pose_cov_stamped.pose, pose_cov_stamped.header);
}

void CovarianceVisualizer::PublishCovariance(
    const nav_msgs::Odometry &odometry) {
  PublishCovariance(odometry.pose, odometry.header);
}

void PoseVisualizer::PublishPose(const geometry_msgs::Pose &pose,
                                 const std_msgs::Header &header) {
  PublishTrajectory(pose.position, header);
  tf_pub_.PublishTransform(pose, header);
}

}  // namespace viz
}  // namespace kr
