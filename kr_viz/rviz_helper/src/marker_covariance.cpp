#include <rviz_helper/kr_marker.hpp>

namespace kr {
namespace viz {
Marker &Marker::covariance(const geometry_msgs::PoseWithCovariance &pose_cov) {
  Eigen::Matrix<double, 6, 6> cov;
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      cov(i, j) = pose_cov.covariance[(i * 6) + j];
    }
  }
  covariance(cov.block<3,3>(0,0));
  position(pose_cov.pose.position);
  return *this;
}

Marker &Marker::covariance(const Eigen::Matrix3d &cov) {
  //  we assume here that covariance is symmetric positive definite
  Eigen::JacobiSVD<Eigen::Matrix3d> SVD = cov.jacobiSvd(Eigen::ComputeFullU);

  Eigen::Matrix3d U = SVD.matrixU();  //  U == V in this case
  Eigen::Vector3d S = SVD.singularValues();

  //  U is a rotation matrix of the ellipse, convert to quaternion
  Eigen::Quaterniond quat(U*U.determinant());
  quat.normalize();

  mark_.pose.orientation.w = quat.w();
  mark_.pose.orientation.x = quat.x();
  mark_.pose.orientation.y = quat.y();
  mark_.pose.orientation.z = quat.z();

  //  set scale
  for (int i = 0; i < 3; i++) {
    S[i] = std::sqrt(S[i]);
  }
  scale(S[0], S[1], S[2]);

  return *this;
}

}  // namespace viz
}  // namespace kr
