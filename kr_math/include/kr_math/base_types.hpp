/*
 * base_types.hpp
 *
 *  Copyright (c) 2014 Kumar Robotics. All rights reserved.
 *
 *  This file is part of kr_math.
 *
 *  Created on: 28/06/2014
 *      Author: gareth
 */

/*
 * @brief Convenient shorthand names for working with vectors/matrices.
 */
#ifndef KR_MATH_BASE_TYPES_H_
#define KR_MATH_BASE_TYPES_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace kr {

template <typename Scalar, int Rows, int Cols>
using Mat = Eigen::Matrix<Scalar, Rows, Cols>;

template <typename Scalar, int Rows>
using Vec = Eigen::Matrix<Scalar, Rows, 1>;

template <typename Scalar> using Vec2 = Mat<Scalar, 2, 1>;

template <typename Scalar> using Vec3 = Mat<Scalar, 3, 1>;

template <typename Scalar> using Vec4 = Mat<Scalar, 4, 1>;

template <typename Scalar> using Mat2 = Mat<Scalar, 2, 2>;

template <typename Scalar> using Mat3 = Mat<Scalar, 3, 3>;

template <typename Scalar> using Mat4 = Mat<Scalar, 4, 4>;

template <typename Scalar> using Quat = Eigen::Quaternion<Scalar>;

typedef Vec2<float> Vec2f;
typedef Vec2<double> Vec2d;

typedef Vec3<float> Vec3f;
typedef Vec3<double> Vec3d;

typedef Vec4<float> Vec4f;
typedef Vec4<double> Vec4d;

typedef Mat2<float> Mat2f;
typedef Mat2<double> Mat2d;

typedef Mat3<float> Mat3f;
typedef Mat3<double> Mat3d;

typedef Mat4<float> Mat4f;
typedef Mat4<double> Mat4d;

typedef Quat<float> Quatf;
typedef Quat<double> Quatd;

} // namespace kr

#endif // KR_MATH_BASE_TYPES_H_
