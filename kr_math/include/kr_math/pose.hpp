/*
 * pose.hpp
 *
 *  Copyright (c) 2013 Kumar Robotics. All rights reserved.
 *
 *  This file is part of kr_math.
 *
 *  Created on: 28/06/2014
 *      Author: gareth
 */

#ifndef KR_MATH_POSE_H_
#define KR_MATH_POSE_H_

/* GTSAM include first since it needs to be included before Eigen as they
 * have a custom Eigen version where they overload some Eigen classes. Having it
 * after the Eigen include prevents their Eigen header being included due to
 * header guards
 */
#ifdef KR_MATH_GTSAM_CONVERSIONS
#include <gtsam/geometry/Pose3.h>
#endif

#include "base_types.hpp"
#include <Eigen/Geometry>

#ifdef KR_MATH_ROS_CONVERSIONS
#include <geometry_msgs/Pose.h>
#endif

namespace kr {

/**
 * @brief Pose, an orientation and position in R3.
 *
 * @note Some important notes on convention:
 *
 * When we write _A to B_ transformation (or transform), it is in reference to
 * the operation effecuated by matrix H, such that:
 *
 * b = H * a, where a is a vector in the A frame and b is that same vector
 * in the B frame.
 *
 * kr::Pose stores `q`, which rotates vectors from body to world frame,
 * and `p`, the center of the body as expressed in the world frame.
 *
 * Transforming a vector v from world to camera (body) is performed with:
 *
 * vb = R(q') * (vw - p), where R(.) maps from S(4) to SO(3).
 *
 * It is important to note that p is equivalent to the body origin in world
 * frame, often denoted wTb.
 *
 */
template <typename Scalar> struct Pose {
private:
  kr::Quat<Scalar> q_; /**< Orientation (body to world transform, or wRb) */
  kr::Vec3<Scalar> p_; /**< Position (origin to body vector) */

public:
  /**
   * @brief Construct with identity pose.
   */
  Pose() : q_(1, 0, 0, 0) { p_[0] = p_[1] = p_[2] = 0; }

  /**
   * @brief Copy-ctor
   */
  Pose(const Pose<Scalar> &pose) : q_(pose.q_), p_(pose.p_) {}

  /**
   * @brief Construct from quaternion and position vector.
   * @param wQb Body to world rotation.
   * @param pInW Origin to body vector.
   */
  Pose(const Quat<Scalar> &wQb, const Vec3<Scalar> &pInW) : q_(wQb), p_(pInW) {}

  /**
   * @brief Construct from a 4x4 homogenous transformation matrix.
   * @param bHw 4x4 matrix representing the body to world transform.
   *
   * @note wHb is expected to perform the operation: vw = wHb * vb.
   */
  Pose(const kr::Mat4<Scalar>& wHb) :
    q_(wHb.template block<3,3>(0,0)), p_(wHb.template block<3,1>(0,3)) {}

#ifdef KR_MATH_ROS_CONVERSIONS
  /**
   * @brief Construct pose from ROS message.
   * @param geo Instance of geometry_msgs::Pose.
   * @note Loss of accuracy will occur if Scalar is float32.
   */
  Pose(const geometry_msgs::Pose &geo) {
    q_ = kr::Quat<Scalar>(geo.orientation.w, geo.orientation.x,
                         geo.orientation.y, geo.orientation.z);
    p_[0] = geo.position.x;
    p_[1] = geo.position.y;
    p_[2] = geo.position.z;
  }

  /**
   * @brief Explicit cast to ROS geometry_msgs::Pose.
   */
  explicit operator geometry_msgs::Pose() const {
    geometry_msgs::Pose geo;
    geo.orientation.w = q_.w();
    geo.orientation.x = q_.x();
    geo.orientation.y = q_.y();
    geo.orientation.z = q_.z();
    geo.position.x = p_[0];
    geo.position.y = p_[1];
    geo.position.z = p_[2];
    return geo;
  }
#endif

#ifdef KR_MATH_GTSAM_CONVERSIONS
  /**
   * @brief Construct pose from GTSAM Pose3.
   * @param gtpose Instance of GTSAM 6DOF pose. GTSAM poses store a body to
   * world transformation.
   */
  Pose(const gtsam::Pose3& gtpose) :
    q_(gtpose.rotation().toQuaternion().cast<Scalar>()),
    p_(gtpose.translation().vector().cast<Scalar>()) {}

  /**
   * @brief Explicit cast to GTSAM Pose3.
   * @note Converts to the GTSAM convention of body to world.
   */
  explicit operator gtsam::Pose3() const {
    const gtsam::Rot3 rot(q_.template cast<double>());
    return gtsam::Pose3(rot, gtsam::Point3(p_.template cast<double>()));
  }
#endif

  /**
   * @brief Position of this pose in the world frame.
   * @return kr::Vec3
   */
  const kr::Vec3<Scalar>& p() const { return p_; }
  kr::Vec3<Scalar>& p() { return p_; }

  /**
   * @brief Quaternion that performs the body to world rotation on vectors.
   * @return kr::Quat
   */
  const kr::Quat<Scalar>& q() const { return q_; }
  kr::Quat<Scalar>& q() { return q_; }

  /**
   * @brief Construct a pose from an rvec/tvec pair.
   *
   * @param rvec Rotation vector. If R = exp(rvec), then R*v will rotate vector
   * v from world to body frame.
   * @param tvec Translation vector, or t = -R*p.
   *
   * @return The corresponding pose.
   *
   * @note The vectors accepted here are in the same form produced by OpenCV.
   */
  static Pose<Scalar> fromVectors(const Vec3<Scalar>& rvec,
                                  const Vec3<Scalar>& tvec) {
    Vec3<Scalar> rnorm(0.0,0.0,0.0);
    const Scalar rn = rvec.norm();
    if (rn > std::numeric_limits<Scalar>::epsilon()*10) {
      rnorm = rvec / rn;
    }
    const Eigen::AngleAxis<Scalar> aa(rn, rnorm);
    Pose<Scalar> pose;
    pose.q_ = kr::Quat<Scalar>(aa).conjugate();
    pose.p_ = -(pose.q_.matrix() * tvec);
    return pose;
  }

  /**
   * @brief Convert this pose so that it is expressed in the body frame of the
   * argument.
   *
   * @param alt Treat as identity reference frame.
   * @return The receiver, as expressed in frame alt.
   */
  Pose<Scalar> expressedIn(const Pose<Scalar> &alt) const {
    const kr::Vec3<Scalar> pn = alt.bRw() * (p_ - alt.p_);
    return Pose(alt.q_.conjugate() * q_, pn);
  }

  /**
   * @brief Generate the inverse transformation.
   * @return Inverse of this pose: [R^T, -R * p]
   *
   * @note If H is the homo. matrix representation of the receiver, then the
   * returned pose will have representation H' where H * H' = eye(4).
   */
  Pose<Scalar> inverse() const {
    return Pose<Scalar>().expressedIn(*this);
  }

  /**
   * @brief Difference between this pose and the argument in world frame.
   * @param alt Pose to subtract from the receiver.
   *
   * @return The difference between the receiver and the argument, as expressed
   * in the world frame.
   */
  Pose<Scalar> difference(const Pose<Scalar> &alt) const {
    return Pose(alt.q_.conjugate() * q_, p_ - alt.p_);
  }

  /**
   * @brief Translation vector of this pose.
   * @return Vector corresponding to: t = -bRw * p.
   */
  kr::Vec3<Scalar> translation() const { return bRw() * -p_; }

  /**
   * @brief Compose a pose onto the receiver.
   * @param rhs Pose to multiply/add onto this one.
   * @return New pose after composition.
   *
   * @note It is assumed that `rhs` expresses a pose in the body frame of the
   * receiver. This operation therefore rotates `rhs` position vector into the
   * world frame prior to addition. Orientation is composed from the right,
   * corresponding to a multiplication in the body frame.
   */
  Pose<Scalar> composeInBody(const Pose<Scalar>& rhs) const {
    return Pose(q_ * rhs.q_, p_ + q_.matrix() * rhs.p_);
  }

  /**
   * @brief Transform a point into the body frame of this pose.
   * @param v Point to transform, expressed in the world frame.
   * @return Point `v` after conversion to the body frame.
   */
  kr::Vec3<Scalar> transformToBody(const kr::Vec3<Scalar>& v) const {
    return bRw() * (v - p_);
  }

  /**
   * @brief Transform a point from the body frame of this pose.
   * @param v Point to transform, expressed in body frame.
   * @return Point `v` after conversion to the world frame.
   */
  kr::Vec3<Scalar> transformFromBody(const kr::Vec3<Scalar>& v) const {
    return wRb()*v + p_;
  }

  /**
   * @brief SE(3) transformation corresponding to this pose.
   * @return 4x4 matrix that performs the body to world transformation: wHb
   */
  kr::Mat4<Scalar> matrix() const {
    kr::Mat4<Scalar> H;
    H.template block<3,3>(0,0) = q_.matrix();
    H.template block<3,1>(0,3) = p_;
    H(3,0) = H(3,1) = H(3,2) = 0;
    H(3,3) = 1;
    return H;
  }

  /**
   * @brief Shorthand for body to world rotation.
   * @note wP = wRb * bP + wTb
   */
  kr::Mat3<Scalar> wRb() const {
    return q_.matrix();
  }

  /**
   * @brief Shorthand for world to body roration.
   * @note bP = bRw * wP + bTw
   */
  kr::Mat3<Scalar> bRw() const {
    return q_.conjugate().matrix();
  }

  /**
   * @brief bTw Translation from body to world in body frame
   * @note bP = bRw * wP + bTw
   * @note Equivalent to the 'translation' vector.
   */
  kr::Vec3<Scalar> bTw() const {
    return translation(); //  -(bRw() * wTb())
  }

  /**
   * @brief Shorthand for translation vector from world origin to body.
   * @note wP = wRb * bP + wTb
   */
  const kr::Vec3<Scalar>& wTb() const {
    return p_;
  }
};

typedef Pose<float> Posef;
typedef Pose<double> Posed;

} // namespace kr

#endif // KR_MATH_POSE_H_
