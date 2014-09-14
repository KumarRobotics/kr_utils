#include "rviz_helper/visualizer.h"

namespace kr {
namespace rviz_helper {

TrajectoryVisualizer::TrajectoryVisualizer(const ros::NodeHandle &nh,
                                           const std::string &topic)
    : nh_{nh},
      traj_pub_(nh_.advertise<visualization_msgs::Marker>(topic, 1)),
      num_skip_(1),
      total_points_cnt_(0) {
  marker_.pose.orientation.w = 1.0;
  set_color(colors::RED);
  set_scale(0.05);
  marker_.action = visualization_msgs::Marker::ADD;
  marker_.type = visualization_msgs::Marker::LINE_STRIP;
  marker_.lifetime = ros::Duration();
}

void TrajectoryVisualizer::PublishTrajectory(const geometry_msgs::Pose &pose,
                                             const std_msgs::Header &header) {
  PublishTrajectory(pose.position, header);
}

void TrajectoryVisualizer::PublishTrajectory(const geometry_msgs::Point &point,
                                             const std_msgs::Header &header) {
  PublishTrajectory(point, header.frame_id, header.stamp);
}

void TrajectoryVisualizer::PublishTrajectory(const geometry_msgs::Point &point,
                                             const std::string &frame_id,
                                             const ros::Time &stamp) {
  ++total_points_cnt_;
  if (total_points_cnt_ % num_skip_) return;
  marker_.points.push_back(point);
  marker_.header.frame_id = frame_id;
  marker_.header.stamp = stamp;
  traj_pub_.publish(marker_);
}

CovarianceVisualizer::CovarianceVisualizer(const ros::NodeHandle &nh,
                                           const std::string &topic)
    : nh_(nh), cov_pub_(nh_.advertise<visualization_msgs::Marker>(topic, 1)) {
  //  default to a beige colour
  set_color({1, 0.9255, 0.5});

  marker_.action = visualization_msgs::Marker::ADD;
  marker_.type = visualization_msgs::Marker::SPHERE;
  marker_.lifetime = ros::Duration();  //  last forever
}

void CovarianceVisualizer::PublishCovariance(
    const geometry_msgs::PoseWithCovariance &pose_cov,
    const std_msgs::Header &header) {
  marker_.header = header;
  marker_.pose.position = pose_cov.pose.position;
  // Should orientation be in body frame?
  marker_.pose.orientation.w = 1;

  double scale[3] = {0, 0, 0};
  for (int i = 0; i < 3; i++) {
    if (pose_cov.covariance[(i * 6) + i] >= 0) {
      scale[i] = std::sqrt(pose_cov.covariance[(i * 6) + i]);
    }
  }

  marker_.scale.x = scale[0];
  marker_.scale.y = scale[1];
  marker_.scale.z = scale[2];
  cov_pub_.publish(marker_);
}

void CovarianceVisualizer::PublishCovariance(
    const geometry_msgs::PoseWithCovarianceStamped &pose_cov_stamped) {
  PublishCovariance(pose_cov_stamped.pose, pose_cov_stamped.header);
}

void CovarianceVisualizer::PublishCovariance(
    const nav_msgs::Odometry &odometry) {
  PublishCovariance(odometry.pose, odometry.header);
}

void TfPublisher::PublishTransform(const geometry_msgs::Pose &pose,
                                   const std_msgs::Header &header) {
  PublishTransform(pose, header.frame_id, header.stamp);
}

void TfPublisher::PublishTransform(const geometry_msgs::Pose &pose,
                                   const std::string &frame_id,
                                   const ros::Time &stamp) {
  geometry_msgs::Vector3 translation;
  translation.x = pose.position.x;
  translation.y = pose.position.y;
  translation.z = pose.position.z;

  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = frame_id;
  transform_stamped.header.stamp = stamp;
  transform_stamped.child_frame_id = child_frame_id_;
  transform_stamped.transform.translation = translation;
  transform_stamped.transform.rotation = pose.orientation;

  tf_broadcaster_.sendTransform(transform_stamped);
}

void PoseVisualizer::PublishPose(const geometry_msgs::Pose &pose,
                                 const std::string &frame_id,
                                 const ros::Time &stamp) {
  traj_viz_.PublishTrajectory(pose.position, frame_id, stamp);
  tf_pub_.PublishTransform(pose, frame_id, stamp);
}

}  // namespace rviz_helper
}  // namespace kr
