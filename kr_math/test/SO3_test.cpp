#include "kr_math/SO3.hpp"
#include <gtest/gtest.h>

#define DOUBLE_NEAR_THRESH (1e-12)

#define EXPECT_CLOSE(a, b) EXPECT_NEAR((a), (b), DOUBLE_NEAR_THRESH)

class SO3Test : public testing::Test {
 protected:
  SO3Test() {
    Rx = kr::rotX(M_PI / 3);
    Ry = kr::rotY(M_PI / 3);
    Rz = kr::rotZ(M_PI / 3);
  }

 protected:
  kr::mat3<double> Rx, Ry, Rz;
};

// Not implemented
TEST_F(SO3Test, RotationXYZ) {

  EXPECT_CLOSE(Rx(0, 0), 1.0);
  EXPECT_CLOSE(Rx(0, 1), 0.0);
  EXPECT_CLOSE(Rx(0, 2), 0.0);
  EXPECT_CLOSE(Rx(1, 0), 0.0);
  EXPECT_CLOSE(Rx(1, 1), 0.5);
  EXPECT_CLOSE(Rx(1, 2), -std::sqrt(3.0) / 2);
  EXPECT_CLOSE(Rx(2, 0), 0.0);
  EXPECT_CLOSE(Rx(2, 1), std::sqrt(3.0) / 2);
  EXPECT_CLOSE(Rx(2, 2), 0.5);

  EXPECT_CLOSE(Ry(0, 0), 0.5);
  EXPECT_CLOSE(Ry(0, 1), 0.0);
  EXPECT_CLOSE(Ry(0, 2), std::sqrt(3.0) / 2);
  EXPECT_CLOSE(Ry(1, 0), 0.0);
  EXPECT_CLOSE(Ry(1, 1), 1.0);
  EXPECT_CLOSE(Ry(1, 2), 0.0);
  EXPECT_CLOSE(Ry(2, 0), -std::sqrt(3.0) / 2);
  EXPECT_CLOSE(Ry(2, 1), 0.0);
  EXPECT_CLOSE(Ry(2, 2), 0.5);

  EXPECT_CLOSE(Rz(0, 0), 0.5);
  EXPECT_CLOSE(Rz(0, 1), -std::sqrt(3.0) / 2);
  EXPECT_CLOSE(Rz(0, 2), 0.0);
  EXPECT_CLOSE(Rz(1, 0), std::sqrt(3.0) / 2);
  EXPECT_CLOSE(Rz(1, 1), 0.5);
  EXPECT_CLOSE(Rz(1, 2), 0.0);
  EXPECT_CLOSE(Rz(2, 0), 0.0);
  EXPECT_CLOSE(Rz(2, 1), 0.0);
  EXPECT_CLOSE(Rz(2, 2), 1.0);
}

TEST_F(SO3Test, RotToEulerZYX) {
  const kr::mat3<double> R = Rz * Ry * Rx;
  const kr::vec3<double> rpy = kr::rotToEulerZYX(R);

  EXPECT_CLOSE(rpy[0], M_PI / 3);
  EXPECT_CLOSE(rpy[1], M_PI / 3);
  EXPECT_CLOSE(rpy[2], M_PI / 3);
}

TEST(RodriguesTest, RodriguesToQuat) {
  const kr::vec3<double> rvec_zero(0, 0, 0);
  Eigen::Quaterniond q0 = kr::rodriguesToQuat(rvec_zero);
  EXPECT_EQ(q0.w(), 1.0);
  EXPECT_EQ(q0.x(), 0.0);
  EXPECT_EQ(q0.y(), 0.0);
  EXPECT_EQ(q0.z(), 0.0);

  const double r = M_PI / std::sqrt(3);
  const kr::vec3<double> rvec(r, r, r);
  Eigen::Quaterniond q = kr::rodriguesToQuat(rvec);
  EXPECT_CLOSE(q.w(), 0.0);
  EXPECT_CLOSE(q.x(), r / M_PI);
}
