/*
 * feature.hpp
 *
 *  Copyright (c) 2014 Kumar Robotics. All rights reserved.
 *
 *  This file is part of kr_vision.
 *
 *  Created on: 06/08/2014
 *     Authors: Gareth Cross, Kartik Mohta
 */

/**
 * @brief Collection of utilities for use with image features.
 */
#ifndef KR_VISION_FEATURE_HPP_
#define KR_VISION_FEATURE_HPP_

#include <kr_math/base_types.hpp>
#include <kr_math/pose.hpp>
#include <kr_math/SO3.hpp>
#include <cmath>

namespace kr {

/**
 * @brief Triangulate by backprojecting rays w/ least squares solution.
 * @param p1 First pose.
 * @param x1 First observation in normalized image coordinates.
 * @param p2 Second pose.
 * @param x2 Second observation in normalized image coordinates.
 *
 * @param pos (output) Position of point in world frame.
 * @param ratio (output) Ratio of largest and smallest Eigenvalues.
 *
 * @return The depth (along camera Z axis) of the feature as measured in p1.
 *
 * @note See "Computer Vision: Algorithms and Applications" (September 3, 2010
 *draft), pp. 346
 * @note See "Vision-Based State Estimation for Autonomous Rotorcraft MAVs in
 *Complex Environments", Shaojie Shen et. al, 2013
 */
template <typename Scalar>
Scalar triangulate(const kr::Pose<Scalar> &p1, const kr::Vec2<Scalar> &x1,
                  const kr::Pose<Scalar> &p2, const kr::Vec2<Scalar> &x2,
                  kr::Vec3<Scalar> &pos, Scalar &ratio) {

  const Mat3<Scalar> R1T = p1.wRb();

  Vec3<Scalar> v1 = R1T * kr::Vec3<Scalar>(x1[0], x1[1], 1);
  Vec3<Scalar> v2 = p2.wRb()* kr::Vec3<Scalar>(x2[0], x2[1], 1);

  v1 /= v1.norm();
  v2 /= v2.norm();

  const Mat3<Scalar> A1 = Mat3<Scalar>::Identity() - v1 * v1.transpose();
  const Mat3<Scalar> A2 = Mat3<Scalar>::Identity() - v2 * v2.transpose();
  const Mat3<Scalar> A = A1 + A2;

  //	calculate eigenvalues
  const Scalar c = A(0, 0) * A(2, 2) - A(2, 0) * A(0, 2) - A(1, 0) * A(0, 1) +
                   A(1, 1) * A(2, 2) + A(0, 0) * A(1, 1) - A(1, 2) * A(2, 1) -
                   4;

  const Scalar lam1 = 2;
  const Scalar lam2 = 1 + std::sqrt(1 - c);
  const Scalar lam3 = 4 - lam1 - lam2;
  const Scalar inv_det = 1 / (lam1*lam2*lam3);
  ratio = lam1 / lam3;

  Mat3<Scalar> Ainv;
  Ainv(0, 0) = (-A(2, 1) * A(1, 2) + A(1, 1) * A(2, 2)) * inv_det;
  Ainv(0, 1) = (-A(0, 1) * A(2, 2) + A(0, 2) * A(2, 1)) * inv_det;
  Ainv(0, 2) = (A(0, 1) * A(1, 2) - A(0, 2) * A(1, 1)) * inv_det;
  Ainv(1, 0) = (A(2, 0) * A(1, 2) - A(1, 0) * A(2, 2)) * inv_det;
  Ainv(1, 1) = (-A(2, 0) * A(0, 2) + A(0, 0) * A(2, 2)) * inv_det;
  Ainv(1, 2) = (A(1, 0) * A(0, 2) - A(0, 0) * A(1, 2)) * inv_det;
  Ainv(2, 0) = (-A(2, 0) * A(1, 1) + A(1, 0) * A(2, 1)) * inv_det;
  Ainv(2, 1) = (A(2, 0) * A(0, 1) - A(0, 0) * A(2, 1)) * inv_det;
  Ainv(2, 2) = (-A(1, 0) * A(0, 1) + A(0, 0) * A(1, 1)) * inv_det;

  //  calculate position
  pos = Ainv * (A1 * p1.p() + A2 * p2.p());

  //  depth in first frame (last column because R1 is transposed)
  const Vec3<Scalar> pf = pos - p1.p();
  return R1T(0, 2) * pf[0] + R1T(1, 2) * pf[1] + R1T(2, 2) * pf[2];
}

/**
 * @brief Triangulate by minimizing the error when going around the triangle
 * formed by the two rays and the translation between the poses.
 * @param p1 First pose.
 * @param x1 First observation in normalized image coordinates.
 * @param p2 Second pose.
 * @param x2 Second observation in normalized image coordinates.
 *
 * @param quality (output) Approximate quality of triangulation, range is [0,1],
 * where 0 is bad and 1 is very good. Values above 0.1 should be good.
 *
 * @return The depth (along the camera Z axis) of the feature as measured in p1.
 * Returns 0 if observations are almost parallel and quality is close to 0.
 *
 * @note Consider d1 and d2 as the depths of the point in frame 1 and 2
 * respectively. We minimize the norm of (d1*x1 - d2*x2 - t) where t is the
 * translation vector between the two poses expressed in the first frame.
 */
template <typename Scalar>
Scalar triangulateSimple(const kr::Pose<Scalar> &p1, const kr::Vec2<Scalar> &x1,
                         const kr::Pose<Scalar> &p2, const kr::Vec2<Scalar> &x2,
                         Scalar &quality)
{
  const kr::Pose<Scalar> p_1_2 = p2.expressedIn(p1);
  kr::Vec3<Scalar> obs1(x1(0), x1(1), 1);
  obs1.normalize();
  kr::Vec3<Scalar> obs2(p_1_2.wRb()*kr::Vec3<Scalar>(x2(0), x2(1), 1));
  obs2.normalize();

  quality = static_cast<Scalar>(1) - obs2.dot(obs1);
  if(quality <= static_cast<Scalar>(10)*std::numeric_limits<Scalar>::epsilon())
    return 0;

  kr::Mat<Scalar, 3, 2> A;
  A << obs1.normalized(), obs2.normalized();
  const kr::Mat2<Scalar> AtA = A.transpose()*A;
  const kr::Vec2<Scalar> depth_vec = AtA.inverse()*A.transpose()*p_1_2.wTb();
  return depth_vec(0)*obs1(2);
}

/**
 * @brief Calculate the std. dev. of the depth estimate from triangulation
 * assuming a 1 pixel error in the image co-ordinates
 * @param depth Computed depth (along camera Z axis) from triangulation
 * @param obs Point in reference image in which depth was calculated
 * @param translation Translation of the camera between the two images expressed
 * in the camera frame of the reference image
 * @param focal_length Focal length of the camera
 *
 * @note See derivation in "REMODE: Probabilistic, Monocular Dense
 * Reconstruction in Real Time", ICRA 2014
 */
template <typename Scalar>
Scalar triangulationDepthSigma(Scalar depth, const kr::Vec2<Scalar> &obs,
                               const kr::Vec3<Scalar> &translation,
                               Scalar focal_length)
{
  const kr::Vec3<Scalar> p(depth*obs(0), depth*obs(1), depth);
  const Scalar t_norm = translation.norm();
  const kr::Vec3<Scalar> t = translation/t_norm;
  const kr::Vec3<Scalar> f = p.normalized();
  const kr::Vec3<Scalar> a = p - translation;
  const Scalar alpha = std::acos(f.dot(t));
  const Scalar beta = std::acos(-a.normalized().dot(t));
  const Scalar pixel_error = 1;
  const Scalar beta_plus = beta + 2*std::atan(pixel_error/(2*focal_length));
  const Scalar gamma = static_cast<float>(M_PI) - alpha - beta_plus;
  const Scalar p_plus_norm = t_norm * std::sin(beta_plus)/std::sin(gamma);
  const Scalar sigma = std::abs(p_plus_norm - p.norm());
  return sigma;
}

/**
 * @brief Calculate covariance of a 3D point, given observation coords, the
 * depth uncertainty, and the uncertainty on individual pixel measurements.
 *
 * @param obsv Observation in normalized coordinates.
 * @param obvsCov Covariance on feature position, in normalized coordinates.J
 * @param depth Depth of the feature in camera frame (local Z).
 * @param depthSigma Std. dev. on feature depth in camera frame.
 *
 * @return 3x3 Covariance matrix.
 *
 * @note obvsSigma is in normalized image plane coordinates. If you wish to
 * assume a standard deviation of s for some focal length f, pass (s/f).
 *
 * @note This method performs the operation J*P*J', where J is the jacobian of
 * [u,v,1]*Z with respect to the normalized image coordinates [u,v] and the
 * feature depth, Z. P is the diagonal covariance of those three quantities.
 */
template <typename Scalar>
kr::Mat3<Scalar> reprojectionCovariance(const kr::Vec2<Scalar>& obsv,
                                        const kr::Mat2<Scalar>& obvsCov,
                                        Scalar depth,
                                        Scalar depthSigma) {
  kr::Mat3<Scalar> J = kr::Mat3<Scalar>::Zero();
  J(0,0) = depth; //  Z in the camera frame
  J(0,2) = obsv[0];
  J(1,1) = depth;
  J(1,2) = obsv[1];
  J(2,2) = 1;
  //  variance, converted to normalized image coordinates
  kr::Mat3<Scalar> P = kr::Mat3<Scalar>::Zero();
  P.template block<2,2>(0,0) = obvsCov;
  P(2,2) = depthSigma*depthSigma;
  return J*P*J.transpose();
}

template <typename Scalar>
kr::Mat3<Scalar> triangulationCovariance(const kr::Vec2<Scalar>& obsv,
                                        Scalar obvsSigma,
                                        Scalar depth,
                                        Scalar depthSigma) {
  kr::Mat3<Scalar> J = kr::Mat3<Scalar>::Zero();
  J(0,0) = depth; //  Z in the camera frame
  J(0,2) = obsv[0];
  J(1,1) = depth;
  J(1,2) = obsv[1];
  J(2,2) = 1;
  //  variance, converted to normalized image coordinates
  kr::Mat3<Scalar> P = kr::Mat3<Scalar>::Zero();
  P(0,0) = P(1,1) = obvsSigma;
  P(2,2) = depthSigma*depthSigma;
  return J*P*J.transpose();
}

/**
 * @brief Refine point position using gauss-newton refinement.
 * @param poses List of poses.
 * @param observations List of observations of feature (normalized space).
 * @param position Initial estimate of feature position in world frame.
 * On output, position contains the refined point - though only if the
 * function returns true.
 *
 * @param weights Weights on each pose. If empty, uniform weights are used.
 * @param maxIterations Max number of iterations allowed.
 * @param epsilon Threshold below which we assume convergence has occurred.
 * @param eta Eta value to use in levenberg marquardt algorithm.
 *
 * @return False if: The hessian is singular during refinement - OR - the
 * feature position proves to be invalid (behind camera plane or at infinity).
 *
 * @throw invalid_argument if less than two poses are provided.
 * @throw invalid_argument if size of weights does not match size of poses.
 *
 * @note This algorithm will refine the position of a point using observations
 * accross multiple frames. The optimization takes place in the reference
 * frame of the first observation, using an inverse depth formulation. These
 * modifications confer additional numerical stability and increase resilience
 * to local minima. Optimization will continue until the update norm falls below
 * epsilon, or maxIterations is reached. Usually 4 or 5 iterations are enough,
 * if a decent initial triangulation is provided.
 *
 * @see "A Multi-State Constraint Kalman Filter for Vision-aided Inertial
 * Navigation" (2007), Mourikis and Roumeliotis.
 */
template <typename Scalar>
bool refinePoint(const std::vector<kr::Pose<Scalar>> &poses,
                 const std::vector<kr::Vec2<Scalar>> &observations,
                 kr::Vec3<Scalar> &position,
                 const std::vector<Scalar>& weights = std::vector<Scalar>(),
                 int maxIterations = 10, Scalar epsilon = 0.001,
                 Scalar eta = 0.005) {

  const size_t numPoses = poses.size();

  if (numPoses < 2) {
    throw std::invalid_argument(
        "2+ observations are required to refine a point");
  }
  if (!weights.empty()) {
    if (weights.size() != poses.size()) {
      throw std::invalid_argument(
          "Number of weights must match number of poses");
    }
  }

  //  feature position in camera frame of pose N (first pose)
  const Pose<Scalar> &poseN = poses.front();
  const Vec3<Scalar> pf_N = poseN.bRw() * (position - poseN.p());

  const Scalar rho = 1 / pf_N[2];
  if (!std::isnormal(rho)) {
    //  the depth is 0 or Inf
    return false;
  }

  //  initial guess for [alpha, beta, rho]
  Vec3<Scalar> params = Vec3<Scalar>(pf_N[0], pf_N[1], 1) * rho;

  Mat<Scalar, Eigen::Dynamic, 1> res(numPoses * 2, 1); //  residuals
  Mat<Scalar, Eigen::Dynamic, 3> jac(numPoses * 2, 3); //  jacobians

  //  weights
  Mat<Scalar, Eigen::Dynamic, Eigen::Dynamic> W(numPoses * 2, numPoses * 2);
  W.setIdentity();
  bool invertible;

  if (!weights.empty()) {
    for (size_t j = 0; j < numPoses; j++) {
      //  calculate weight of this measurement
      W(j * 2, j * 2) = W(j * 2 + 1, j * 2 + 1) = weights[j];
    }
  }

  Mat<Scalar, 2, 3> J1 = Mat<Scalar, 2, 3>::Zero();
  Mat<Scalar, 3, 3> J2;
  Mat<Scalar, 2, 3> J;

  //  calculate relative poses
  std::vector<Mat3<Scalar>> jRn;
  std::vector<Vec3<Scalar>> jTn;

  jRn.reserve(numPoses);
  jTn.reserve(numPoses);

  for (size_t i = 0; i < numPoses; i++) {
    const Pose<Scalar> &poseJ = poses[i];
    const Pose<Scalar> jInN = poseJ.expressedIn(poseN);

    jRn.push_back(jInN.bRw());  //  matrix from N to J
    jTn.push_back(poseJ.bRw() * (poseJ.p() - poseN.p()));
  }

  for (int i = 0; i < maxIterations; i++) {
    for (size_t j = 0; j < numPoses; j++) {

      //  pose J expressed in pose N
      const Mat3<Scalar> &R = jRn[j];
      const Vec3<Scalar> &p = jTn[j];
      const Vec2<Scalar> &img = observations[j];
      //  conversion from pose N to pose J
      const Vec3<Scalar> h =
          R * Vec3<Scalar>(params[0], params[1], 1) - (params[2] * p);
      const Scalar invz = 1 / h[2];

      //  projection jacobian
      J1(0, 0) = invz;
      J1(0, 2) = -h[0] * invz * invz;
      J1(1, 1) = invz;
      J1(1, 2) = -h[1] * invz * invz;

      //  transformation jacobian
      for (int n = 0; n < 3; n++) {
        J2(n, 0) = R(n, 0);
        J2(n, 1) = R(n, 1);
        J2(n, 2) = -p[n];
      }

      J = J1 * J2;
      for (int n = 0; n < 3; n++) {
        jac(j * 2 + 0, n) = J(0, n);
        jac(j * 2 + 1, n) = J(1, n);
      }

      //  residual
      res[j * 2 + 0] = (img[0] - h[0] * invz) * W(j * 2, j * 2);
      res[j * 2 + 1] = (img[1] - h[1] * invz) * W(j * 2 + 1, j * 2 + 1);
    }

    auto jacT = jac.transpose();
    Mat3<Scalar> H = jacT * W * jac; //  weighted hessian

    //  levenberg component
    for (int i = 0; i < 3; i++) {
      H(i, i) *= (1 + eta);
    }

    Mat3<Scalar> Hinv;
    H.computeInverseWithCheck(Hinv, invertible);

    if (invertible) {
      //  calculate correction to alpha, beta and rho
      const Vec3<Scalar> cor = Hinv * (jacT * res);

      for (int n = 0; n < 3; n++) {
        params[n] += cor[n];
      }

      const Scalar mag_2 = cor[0] * cor[0] + cor[1] * cor[1] + cor[2] * cor[2];
      if (mag_2 < epsilon * epsilon) {
        //  system has converged, stop
        break;
      }
    } else {
      //  diverged or ill-conditioned
      return false;
    }
  }

  const Scalar invz = params[2];
  if (invz < 1e-3) {
    //  feature is approximately at infinity, or on wrong size of camera plane
    // (invz < 0)
    return false;
  }

  //  convert back to world coordinates
  position = poseN.wRb() * Vec3<Scalar>(params[0], params[1], 1)/invz
      + poseN.p();
  return true;
}

/**
 * @brief Probabilistic filter for outlier rejection in depth estimation.
 *
 * @note A recursive filter for estimating the depth of a feature. Produces a
 * value for the depth, as well as a probability that the feature is an inlier (
 * pi). It is recommended you read the paper below for a full description
 * of the assumptions and operation of the filter.
 *
 * @see "Video-based, real-time multi-view stereo", G. Vogiatzis, C. Hernandez,
 * (2011)
 */
template <typename Scalar> struct DepthFilter {

  Scalar A, B;      /// Beta distribution parameters.
  Scalar mu, sigma; /// Normal distribution parameters.

  Scalar minZ, maxZ; /// Measurement uniform distribution parameters.

  /**
   * @brief Default constructor initializes all values to zero.
   */
  DepthFilter() : A(0), B(0), mu(0), sigma(0), minZ(0), maxZ(0) {}

  /**
   * @brief initialize
   * @param minDepth Minimum measureable depth.
   * @param maxDepth Maximum measureable depth.
   *
   * @note Initializes pi to 50%, and depth to half-way between min/maxZ. Sigma
   * is initialized so that 99.7% of the normal probability mass lies in the
   * interval [mindepth, maxDepth].
   */
  void initialize(Scalar minDepth, Scalar maxDepth) {
    minZ = minDepth;
    maxZ = maxDepth;
    A = B = 10;
    mu = (minZ + maxZ) / 2;
    sigma = (maxZ - minZ)/6;
  }

  /**
   * @brief Update the filter with the addition of a new depth estimate.
   * @param Xn Depth to the feature in the reference frame.
   * @param measStd Std dev. of noise on depth measurement.
   *
   * @note It is assumed that all depth estimates are from the same reference
   * frame. This function updates the parameters of the distribution by moment
   * matching. Derivation provided in Vogiatzis and Hernandez (2011).
   *
   * @return False if the probability of the measurement is effectively zero.
   */
  bool addMeasurement(Scalar Xn, Scalar measStd) {
    const Scalar tau = measStd;

    //  update the first and second moments of the depth distribution
    const Scalar nsig = std::sqrt(sigma * sigma + tau * tau);
    Scalar C1 = normpdf(Xn, mu, nsig) * A / (A + B);
    Scalar C2 = unifpdf(Xn, minZ, maxZ) * B / (A + B);

    const Scalar total = C1 + C2;
    if (total < 1e-6) {
      return false;
    } else {
      C1 /= total;
      C2 /= total;

      const Scalar G1 = (Xn * sigma * sigma + mu * tau * tau) / (nsig * nsig);
      const Scalar G2 = (sigma * sigma * tau * tau) / (nsig * nsig);

      //  new mu and sigma
      const Scalar nmu = G1 * C1 + C2 * mu;
      const Scalar sigp = std::sqrt(C1 * (G1 * G1 + G2) +
                                    C2 * (mu * mu + sigma * sigma) - nmu * nmu);
      mu = nmu;
      sigma = sigp;

      //  new A & B
      const Scalar K1 = C1 * (1 + A) / (1 + A + B) + C2 * A / (1 + A + B);
      const Scalar K2 = C1 * (1 + A) * (2 + A) / ((1 + A + B) * (2 + A + B)) +
                        C2 * A * (1 + A) / ((1 + A + B) * (2 + A + B));

      A = -K1 * (-K2 + K1) / (-K2 + K1 * K1);
      B = (K1 - 1) * (-K2 + K1) / (-K2 + K1 * K1);
    }
    return true;
  }

  /**
   * @brief Get value of a given depth on the filter's normal distribution.
   * @param X Depth to calculate.
   * @return The gaussian pdf of X conditioned on distribution N(mu, sigma).
   */
  Scalar depthPdf(Scalar X) const {
    return normpdf(X,mu,sigma);
  }

  /**
   * @brief Current depth estimate in the filter.
   * @return mu
   */
  Scalar depth() const { return mu; }

  /**
   * @brief Probability of the feature being an inlier.
   * @return pi
   */
  Scalar pi() const { return A / (A + B); }

private:
  static Scalar unifpdf(Scalar x, Scalar xmin, Scalar xmax) {
    if (x < xmin || x > xmax) {
      return 0;
    }
    return 1 / (xmax - xmin);
  }

  static Scalar normpdf(Scalar x, Scalar mu, Scalar sigma) {
    static const Scalar denom = std::sqrt(2 * static_cast<Scalar>(M_PI));
    return std::exp(-(x - mu) * (x - mu) / (2 * sigma * sigma)) /
           (sigma * denom);
  }
};
}

#endif // KR_VISION_FEATURE_HPP_
