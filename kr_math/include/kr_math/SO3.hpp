/*
 * SO3.hpp
 *
 *  Copyright (c) 2014 Kumar Robotics. All rights reserved.
 *
 *  This file is part of kr_math.
 *
 *  Created on: 22/6/2014
 *      Author: gareth
 */

/*
 * @brief SO3.hpp Collection of methods useful when operating on SO3
 */
#ifndef KR_MATH_SO3_HPP_
#define KR_MATH_SO3_HPP_

#include <cmath>
#include "base_types.hpp"

namespace kr {

/**
 * @brief Get the roll,pitch,yaw angles from a rotation matrix.
 * @param R Member of SO(3).
 * @return 3x1 Vector with elements [roll,pitch,yaw] about [x,y,z] axes.
 *
 * @note Assumes matrix is of the form: (world) = Rz * Ry * Rx (body).
 */
template <typename T>
kr::Vec3<T> rotToEulerZYX(const kr::Mat3<T>& R) {
  T sth = -R(2, 0);
  if (sth > 1) {
    sth = 1;
  } else if (sth < -1) {
    sth = -1;
  }

  const T theta = std::asin(sth);
  const T cth = std::sqrt(static_cast<T>(1) - sth * sth);

  T phi, psi;
  if (cth < std::numeric_limits<T>::epsilon() * 10) {
    phi = std::atan2(R(0, 1), R(1, 1));
    psi = 0;
  } else {
    phi = std::atan2(R(2, 1), R(2, 2));
    psi = std::atan2(R(1, 0), R(0, 0));
  }

  kr::Vec3<T> rpy;
  rpy[0] = phi;    //  x, [-pi,pi]
  rpy[1] = theta;  //  y, [-pi/2,pi/2]
  rpy[2] = psi;    //  z, [-pi,pi]
  return rpy;
}

/**
 * @brief Get the roll, pitch, yaw angles from a quaternion.
 * @param q Quaternion.
 * @return 3x1 Vector with elements [roll,pitch,yaw] about [x,y,z] axes.
 *
 * @note Assumes quaternion represents rotation of the form:
 * (world) = Rz * Ry * Rx (body).
 */
template <typename T>
kr::Vec3<T> quatToEulerZYX(const kr::Quat<T>& q) {
  T q0 = q.w(), q1 = q.x(), q2 = q.y(), q3 = q.z();

  T sth = 2 * (q0 * q2 - q1 * q3);
  if (sth > 1) {
    sth = 1;
  } else if (sth < -1) {
    sth = -1;
  }

  const T theta = std::asin(sth);
  const T cth = std::sqrt(static_cast<T>(1) - sth * sth);

  T phi, psi;
  if(cth < std::numeric_limits<T>::epsilon() * 10) {
    phi = std::atan2(2 * (q1*q2 - q0*q3), q0*q0 - q1*q1 + q2*q2 - q3*q3);
    psi = 0;
  } else {
    phi = std::atan2(2 * (q0*q1 + q2*q3), q0*q0 - q1*q1 - q2*q2 + q3*q3);
    psi = std::atan2(2 * (q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3);
  }

  kr::Vec3<T> rpy;
  rpy[0] = phi;    //  x, [-pi,pi]
  rpy[1] = theta;  //  y, [-pi/2,pi/2]
  rpy[2] = psi;    //  z, [-pi,pi]
  return rpy;
}

/**
 * @brief X rotation matrix.
 * @param angle Angle in radians.
 * @return 3x3 member of SO(3) corresponding to a CCW rotation
 *   about the x-axis.
 */
template <typename T>
kr::Mat3<T> rotX(T angle) {
  const T c = std::cos(angle);
  const T s = std::sin(angle);

  kr::Mat3<T> R;

  R(0, 0) = 1;
  R(0, 1) = R(0, 2) = 0;
  R(1, 0) = 0;
  R(1, 1) = c;
  R(1, 2) = -s;
  R(2, 0) = 0;
  R(2, 1) = s;
  R(2, 2) = c;

  return R;
}

/**
 * @brief Y rotation matrix
 * @param angle Angle in radians
 * @return 3x3 member of SO(3) corresponding to a CCW rotation
 *   about the y axis.
 */
template <typename T>
kr::Mat3<T> rotY(T angle) {
  const T c = std::cos(angle);
  const T s = std::sin(angle);

  kr::Mat3<T> R;

  R(1, 1) = 1;
  R(1, 0) = R(1, 2) = 0;
  R(0, 0) = c;
  R(0, 1) = 0;
  R(0, 2) = s;
  R(2, 0) = -s;
  R(2, 1) = 0;
  R(2, 2) = c;

  return R;
}

/**
 * @brief Z rotation matrix.
 * @param angle Angle in radians.
 * @return 3x3 member of SO(3) corresponding to a CCW rotation
 *   about the z axis.
 */
template <typename T>
kr::Mat3<T> rotZ(T angle) {
  const T c = std::cos(angle);
  const T s = std::sin(angle);

  kr::Mat3<T> R;

  R(2, 2) = 1;
  R(2, 0) = R(2, 1) = 0;
  R(0, 0) = c;
  R(0, 1) = -s;
  R(0, 2) = 0;
  R(1, 0) = s;
  R(1, 1) = c;
  R(1, 2) = 0;

  return R;
}

/**
 *  @brief Create a skew-symmetric matrix from a 3-element vector.
 *  @note Performs the operation:
 *  w   ->  [  0 -w3  w2]
 *          [ w3   0 -w1]
 *          [-w2  w1   0]
 */
template <typename Scalar>
kr::Mat3<Scalar> skewSymmetric(const kr::Vec3<Scalar>& w) {
  kr::Mat3<Scalar> W;
  W(0, 0) = 0;
  W(0, 1) = -w(2);
  W(0, 2) = w(1);
  W(1, 0) = w(2);
  W(1, 1) = 0;
  W(1, 2) = -w(0);
  W(2, 0) = -w(1);
  W(2, 1) = w(0);
  W(2, 2) = 0;
  return W;
}

/**
 *  @brief Create a (squared) skew-symmetric matrix from a 3-element vector.
 *  @note Performs the operation:
 *  w ->  S(w)^2
 */
template <typename Scalar>
kr::Mat3<Scalar> skewSymmetric2(const kr::Vec3<Scalar>& w) {
  kr::Mat3<Scalar> W2;
  W2(0, 0) = -(w[2] * w[2] + w[1] * w[1]);
  W2(1, 1) = -(w[2] * w[2] + w[0] * w[0]);
  W2(2, 2) = -(w[1] * w[1] + w[0] * w[0]);
  W2(1, 0) = W2(0, 1) = w[0] * w[1];
  W2(2, 0) = W2(0, 2) = w[0] * w[2];
  W2(1, 2) = W2(2, 1) = w[1] * w[2];
  return W2;
}

/**
 * @brief rodriguesToQuat Convert rodrigues parameters to quaternion
 * @return Unit quaternion representing the same rotation
 */
template <typename Scalar>
kr::Quat<Scalar> rodriguesToQuat(const kr::Vec3<Scalar>& r) {
  const Scalar r_norm = r.norm();
  kr::Vec3<Scalar> r_normalized(0.0, 0.0, 0.0);
  if (r_norm > std::numeric_limits<Scalar>::epsilon() * 10) {
    r_normalized = r / r_norm;
  }
  return kr::Quat<Scalar>(Eigen::AngleAxis<Scalar>(r_norm, r_normalized));
}

template <typename Scalar>
kr::Quat<Scalar> rodriguesToQuat(Scalar rx, Scalar ry, Scalar rz) {
  return rodriguesToQuat(kr::Vec3<Scalar>(rx, ry, rz));
}

/**
 * @brief Get quaternion given the roll, pitch, yaw angles.
 * @param roll Rotation angle about X axis.
 * @param pitch Rotation angle about Y axis.
 * @param yaw Rotation angle about Z axis.
 * @return Quaternion representing rotation of the form:
 *   (world) = Rz(yaw) * Ry(pitch) * Rx(roll) * (body).
 */
template <typename Scalar>
kr::Quat<Scalar> eulerZYXToQuat(Scalar roll, Scalar pitch, Scalar yaw) {
  Scalar c1 = std::cos(roll/2), s1 = std::sin(roll/2);
  Scalar c2 = std::cos(pitch/2), s2 = std::sin(pitch/2);
  Scalar c3 = std::cos(yaw/2), s3 = std::sin(yaw/2);

  kr::Quat<Scalar> q;
  q.w() = c1*c2*c3 + s1*s2*s3;
  q.x() = s1*c2*c3 - c1*s2*s3;
  q.y() = c1*s2*c3 + s1*c2*s3;
  q.z() = c1*c2*s3 - s1*s2*c3;

  return q;
}

/**
 * @brief Get rotation matrix given the roll, pitch, yaw angles.
 * @param roll Rotation angle about X axis.
 * @param pitch Rotation angle about Y axis.
 * @param yaw Rotation angle about Z axis.
 * @return Rotation matrix representing rotation of the form:
 *   (world) = Rz(yaw) * Ry(pitch) * Rx(roll) * (body).
 */
template <typename Scalar>
kr::Mat3<Scalar> eulerZYXToRot(Scalar roll, Scalar pitch, Scalar yaw) {
  const Scalar c1 = std::cos(roll), s1 = std::sin(roll);
  const Scalar c2 = std::cos(pitch), s2 = std::sin(pitch);
  const Scalar c3 = std::cos(yaw), s3 = std::sin(yaw);

  kr::Mat3<Scalar> R;
  R << c2*c3, -c1*s3 + s1*s2*c3,  s1*s3 + c1*s2*c3,
       c2*s3,  c1*c3 + s1*s2*s3, -s1*c3 + c1*s2*s3,
         -s2,             s1*c2,             c1*c2;

  return R;
}

}  // namespace kr

#endif  // KR_MATH_SO3_HPP_
