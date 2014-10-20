#include "kr_math/pose.hpp"
#include <gtest/gtest.h>

TEST(PoseTest, IdentityConstructor) {
  kr::Pose<double> pose;
  EXPECT_DOUBLE_EQ(pose.q().x(), 0);
  EXPECT_DOUBLE_EQ(pose.q().y(), 0);
  EXPECT_DOUBLE_EQ(pose.q().z(), 0);
  EXPECT_DOUBLE_EQ(pose.q().w(), 1);
  EXPECT_DOUBLE_EQ(pose.p().x(), 0);
  EXPECT_DOUBLE_EQ(pose.p().y(), 0);
  EXPECT_DOUBLE_EQ(pose.p().z(), 0);
}
