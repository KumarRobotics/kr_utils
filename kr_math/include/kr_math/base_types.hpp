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
using mat = Eigen::Matrix<Scalar, Rows, Cols>;

template <typename Scalar, int Rows>
using vec = Eigen::Matrix<Scalar, Rows, 1>;

template <typename Scalar> using vec2 = mat<Scalar, 2, 1>;

template <typename Scalar> using vec3 = mat<Scalar, 3, 1>;

template <typename Scalar> using vec4 = mat<Scalar, 4, 1>;

template <typename Scalar> using mat2 = mat<Scalar, 2, 2>;

template <typename Scalar> using mat3 = mat<Scalar, 3, 3>;

template <typename Scalar> using mat4 = mat<Scalar, 4, 4>;

template <typename Scalar> using quat = Eigen::Quaternion<Scalar>;

typedef vec2<float> vec2f;
typedef vec2<double> vec2d;

typedef vec3<float> vec3f;
typedef vec3<double> vec3d;

typedef vec4<float> vec4f;
typedef vec4<double> vec4d;

typedef mat2<float> mat2f;
typedef mat2<double> mat2d;

typedef mat3<float> mat3f;
typedef mat3<double> mat3d;

typedef mat4<float> mat4f;
typedef mat4<double> mat4d;

typedef quat<float> quatf;
typedef quat<double> quatd;

} // namespace kr

#endif // KR_MATH_BASE_TYPES_H_
