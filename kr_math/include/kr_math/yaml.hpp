/*
 * yaml.hpp
 *
 *  Copyright (c) 2014 Kumar Robotics. All rights reserved.
 *
 *  This file is part of kr_math.
 *
 *  Created on: 16/07/2014
 *      Author: gareth
 */

/*
 * @brief yaml-cpp Conversions for common Eigen types.
 */
#ifndef KR_MATH_YAML_H_
#define KR_MATH_YAML_H_

#ifdef KR_MATH_YAMLCPP_CONVERSIONS

#include <kr_math/base_types.hpp>
#include <yaml-cpp/yaml.h>

namespace kr {

/**
 * @brief Encode a static matrix into yaml.
 * @param M Matrix to encode.
 * @return Instance of YAML::Node.
 */
template <typename Scalar, int Rows, int Cols>
static YAML::Node encodeMat(const kr::Mat<Scalar, Rows, Cols> &M) {
  static_assert(Rows != Eigen::Dynamic && Cols != Eigen::Dynamic,
                "Static matrices only");

  YAML::Node node;
  for (int i = 0; i < Rows; i++) {
    for (int j = 0; j < Cols; j++) {
      node[i*Cols + j] = static_cast<double>(M(i, j));
    }
  }
  return node;
}

/**
 * @brief Decode a static matrix from a yaml sequence.
 * @param node Node to search, must match the size of the matrix.
 * @param M Output matrix.
 * @return true if decoding is successful, false otherwise.
 */
template <typename Scalar, int Rows, int Cols>
static bool decodeMat(const YAML::Node &node, kr::Mat<Scalar, Rows, Cols> &M) {
  static_assert(Rows != Eigen::Dynamic && Cols != Eigen::Dynamic,
                "Static matrices only");

  if (!node.IsSequence() || node.size() != Rows*Cols) {
    return false;
  }

  for (int i = 0; i < Rows; i++) {
    for (int j = 0; j < Cols; j++) {
      M(i, j) = static_cast<Scalar>(node[i * Cols + j].as<double>());
    }
  }
  return true;
}
} // namespace kr


/**
 * @brief Emitter operator for static-size matrices.
 * @param out Emitter to encode a sequence to.
 * @param M Matrix to emit.
 */
template <typename Scalar, int Rows, int Cols>
YAML::Emitter& operator << (YAML::Emitter& out, const kr::Mat<Scalar,Rows,Cols>& M) {
  static_assert(Rows != Eigen::Dynamic && Cols != Eigen::Dynamic,
                "Static matrices only");

  out << YAML::Flow;
  out << YAML::BeginSeq;
  for (int i=0; i < Rows; i++) {
    for (int j=0; j < Cols; j++) {
      out << static_cast<double>(M(i,j));
    }
  }
  out << YAML::EndSeq;
  return out;
}

#define DECLARE_CONVERT(cname, sname)                                          \
  template <> struct convert<cname<sname>> {                                   \
    static Node encode(const cname<sname> &rhs) { return kr::encodeMat(rhs); } \
    static bool decode(const Node &node, cname<sname> &rhs) {                  \
      return kr::decodeMat(node, rhs);                                         \
    }                                                                          \
  }

namespace YAML {

/*
 *  Common vector and matrix types.
 */

DECLARE_CONVERT(kr::Vec2, float);
DECLARE_CONVERT(kr::Vec2, double);
DECLARE_CONVERT(kr::Vec3, float);
DECLARE_CONVERT(kr::Vec3, double);
DECLARE_CONVERT(kr::Vec4, float);
DECLARE_CONVERT(kr::Vec4, double);

DECLARE_CONVERT(kr::Mat2, float);
DECLARE_CONVERT(kr::Mat2, double);
DECLARE_CONVERT(kr::Mat3, float);
DECLARE_CONVERT(kr::Mat3, double);
DECLARE_CONVERT(kr::Mat4, float);
DECLARE_CONVERT(kr::Mat4, double);
} // namespace yaml

#endif // KR_MATH_YAMLCPP_CONVERSIONS
#endif // KR_MATH_YAML_H_
