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
kr::vec3<T> rotToEulerZYX(const kr::mat3<T>& R) {
  kr::vec3<T> rpy;

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
kr::mat3<T> rotX(T angle) {
  const T c = std::cos(angle);
  const T s = std::sin(angle);

  kr::mat3<T> R;

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
kr::mat3<T> rotY(T angle) {
  const T c = std::cos(angle);
  const T s = std::sin(angle);

  kr::mat3<T> R;

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
kr::mat3<T> rotZ(T angle) {
  const T c = std::cos(angle);
  const T s = std::sin(angle);

  kr::mat3<T> R;

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
 *  w   ->  [  0 -w2  w1]
 *          [ w2   0 -w3]
 *          [-w1  w3   0]
 */
template <typename Scalar>
kr::mat3<Scalar> skewSymmetric(const kr::vec3<Scalar>& w) {
  kr::mat3<Scalar> W;
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
kr::mat3<Scalar> skewSymmetric2(const kr::vec3<Scalar>& w) {
  kr::mat3<Scalar> W2;
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
 * @return Unit quaternion rerpresenting the same rotation
 */
template <typename Scalar>
Eigen::Quaternion<Scalar> rodriguesToQuat(const kr::vec3<Scalar>& r) {
  const Scalar r_norm = r.norm();
  kr::vec3<Scalar> r_normalized(0.0, 0.0, 0.0);
  if (r_norm > std::numeric_limits<Scalar>::epsilon() * 10) {
    r_normalized = r / r_norm;
  }
  return Eigen::Quaternion<Scalar>(
      Eigen::AngleAxis<Scalar>(r_norm, r_normalized));
}

template <typename Scalar>
Eigen::Quaternion<Scalar> rodriguesToQuat(Scalar rx, Scalar ry, Scalar rz) {
  return rodriguesToQuat(kr::vec3<Scalar>(rx, ry, rz));
}

}  // namespace kr

#endif  // KR_MATH_SO3_HPP_
