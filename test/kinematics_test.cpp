// Copyright 2026 taoist162-cyber
#include "hero_chassis_controller/mecanum_kinematics.h"

#include <limits>
#include <cmath>
#include <stdexcept>

#include <gtest/gtest.h>

namespace hero_chassis_controller
{
TEST(MecanumKinematics, UnitForwardLeftAndCounterclockwise)
{
  MecanumKinematics kinematics(0.1, 0.4, 0.6);
  const auto forward = kinematics.inverse({1.0, 0.0, 0.0});
  const auto left = kinematics.inverse({0.0, 1.0, 0.0});
  const auto rotation = kinematics.inverse({0.0, 0.0, 1.0});
  const std::array<double, 4> left_expected{{-10.0, 10.0, 10.0, -10.0}};
  const std::array<double, 4> rotation_expected{{-5.0, 5.0, -5.0, 5.0}};
  for (std::size_t i = 0; i < 4; ++i)
  {
    EXPECT_DOUBLE_EQ(10.0, forward[i]);
    EXPECT_DOUBLE_EQ(left_expected[i], left[i]);
    EXPECT_DOUBLE_EQ(rotation_expected[i], rotation[i]);
  }
}

TEST(MecanumKinematics, MixedTranslationAndRotation)
{
  MecanumKinematics kinematics(0.1, 0.4, 0.6);
  const auto wheels = kinematics.inverse({0.5, -0.2, 0.6});
  const std::array<double, 4> expected{{4.0, 6.0, 0.0, 10.0}};
  for (std::size_t i = 0; i < 4; ++i)
  {
    EXPECT_NEAR(expected[i], wheels[i], 1e-12);
  }
}

TEST(MecanumKinematics, RejectsInvalidGeometry)
{
  EXPECT_THROW(MecanumKinematics(0.0, 0.4, 0.4), std::invalid_argument);
  EXPECT_THROW(MecanumKinematics(0.1, -0.4, 0.4), std::invalid_argument);
  EXPECT_THROW(MecanumKinematics(0.1, 0.4, std::numeric_limits<double>::infinity()), std::invalid_argument);
}
TEST(MecanumKinematics, ForwardMatchesMeasuredWheelDirections)
{
  MecanumKinematics kinematics(0.1, 0.4, 0.6);
  const auto forward = kinematics.forward({{10.0, 10.0, 10.0, 10.0}});
  EXPECT_DOUBLE_EQ(1.0, forward.x);
  EXPECT_DOUBLE_EQ(0.0, forward.y);
  EXPECT_DOUBLE_EQ(0.0, forward.yaw);
  const auto left = kinematics.forward({{-10.0, 10.0, 10.0, -10.0}});
  EXPECT_DOUBLE_EQ(1.0, left.y);
  const auto rotation = kinematics.forward({{-5.0, 5.0, -5.0, 5.0}});
  EXPECT_DOUBLE_EQ(1.0, rotation.yaw);
}

TEST(MecanumKinematics, MixedRoundTrip)
{
  MecanumKinematics kinematics;
  const PlanarVelocity expected{0.7, -0.2, 1.1};
  const auto measured = kinematics.forward(kinematics.inverse(expected));
  EXPECT_NEAR(expected.x, measured.x, 1e-12);
  EXPECT_NEAR(expected.y, measured.y, 1e-12);
  EXPECT_NEAR(expected.yaw, measured.yaw, 1e-12);
}

TEST(WheelOdometry, StraightLateralAndInvalidPeriods)
{
  WheelOdometry odometry;
  odometry.update({1.0, 0.5, 0.0}, 2.0);
  EXPECT_DOUBLE_EQ(2.0, odometry.getPose().x);
  EXPECT_DOUBLE_EQ(1.0, odometry.getPose().y);
  odometry.update({10.0, 10.0, 10.0}, 0.0);
  odometry.update({10.0, 10.0, 10.0}, -1.0);
  odometry.update({10.0, 10.0, 10.0}, std::numeric_limits<double>::quiet_NaN());
  EXPECT_DOUBLE_EQ(2.0, odometry.getPose().x);
  EXPECT_DOUBLE_EQ(1.0, odometry.getPose().y);
  odometry.reset();
  EXPECT_DOUBLE_EQ(0.0, odometry.getPose().x);
}

TEST(WheelOdometry, ExactQuarterCircleAndBodyToWorldRotation)
{
  const double half_pi = std::acos(-1.0) / 2.0;
  WheelOdometry odometry;
  odometry.update({1.0, 0.0, 1.0}, half_pi);
  EXPECT_NEAR(1.0, odometry.getPose().x, 1e-12);
  EXPECT_NEAR(1.0, odometry.getPose().y, 1e-12);
  EXPECT_NEAR(half_pi, odometry.getPose().yaw, 1e-12);
  odometry.update({1.0, 0.0, 0.0}, 1.0);
  EXPECT_NEAR(1.0, odometry.getPose().x, 1e-12);
  EXPECT_NEAR(2.0, odometry.getPose().y, 1e-12);
}

TEST(WheelOdometry, ContinuousNearZeroRotation)
{
  WheelOdometry odometry;
  odometry.update({1.0, 0.2, 1e-10}, 1.0);
  EXPECT_NEAR(1.0, odometry.getPose().x, 1e-9);
  EXPECT_NEAR(0.2, odometry.getPose().y, 1e-9);
}
}  // namespace hero_chassis_controller

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
