#include "rviz_helper/marker_visualizer.hpp"

#include <Eigen/Dense>
#include <Eigen/SVD>

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
                                           const std::string &topic, 
                                           bool transforms_orientation)
    : MarkerVisualizer(nh, topic) {
  //  default to a beige colour
  SetColor(1, 0.9255, 0.5);
  SetAction(Marker::ADD);
  SetType(Marker::SPHERE);
  SetLifetime(ros::Duration());  // last forever
  transform_orientation_ = transforms_orientation;
}

void CovarianceVisualizer::PublishCovariance(const PoseWithCovariance &pose_cov,
                                             const std_msgs::Header &header) {
  marker_.header = header;
  marker_.pose.position = pose_cov.pose.position;
  if (!transform_orientation_) {
    marker_.pose.orientation.w = 1;
  
    double scale[3] = {0, 0, 0};
    for (int i = 0; i < 3; i++) {
      if (pose_cov.covariance[(i * 6) + i] >= 0) {
        scale[i] = std::sqrt(pose_cov.covariance[(i * 6) + i]);
      }
    }
  
    SetScale(scale);
  } else {
    using namespace Eigen;
    //  we will automatically determine scale and orientation of the covariance
    
    Matrix<double,6,6> cov;
    for (int i=0; i < 6; i++) {
      for (int j=0; j < 6; j++) {
        cov(i,j) = pose_cov.covariance[(i*6) + j];
      }
    }
    //  position components
    Matrix3d cov_p = cov.template topLeftCorner<3,3>();
    //  we assume here that covariance is symmetric positive definite
    JacobiSVD<Matrix3d> SVD = cov_p.jacobiSvd(Eigen::ComputeFullU);
    
    Matrix3d U = SVD.matrixU();         //  U == V in this case
    Vector3d S = SVD.singularValues();
    
    //  U is a rotation matrix of the ellipse, convert to quaternion
    Eigen::Quaterniond quat(U);
    quat.normalize();
    
    marker_.pose.orientation.w = quat.w();
    marker_.pose.orientation.x = quat.x();
    marker_.pose.orientation.y = quat.y();
    marker_.pose.orientation.z = quat.z();
    
    //  set scale
    for (int i=0; i < 3; i++) {
      S[i] = std::sqrt(S[i]);
    }
    SetScale(S[0], S[1], S[2]);
  }
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
