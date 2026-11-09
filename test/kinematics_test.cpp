// Copyright 2026 taoist162-cyber
#include "hero_chassis_controller/mecanum_kinematics.h"

#include <limits>
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
}  // namespace hero_chassis_controller

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
