// Copyright 2026 taoist162-cyber
#include "hero_chassis_controller/mecanum_kinematics.h"

#include <cmath>
#include <stdexcept>

namespace hero_chassis_controller
{
MecanumKinematics::MecanumKinematics(double wheel_radius, double wheel_base, double wheel_track)
  : wheel_radius_(wheel_radius), rotation_lever_(0.5 * wheel_base + 0.5 * wheel_track)
{
  for (double value : {wheel_radius, wheel_base, wheel_track})
  {
    if (!std::isfinite(value) || value <= 0.0)
    {
      throw std::invalid_argument("Wheel radius, wheel base and track must be finite and positive.");
    }
  }
}

std::array<double, 4> MecanumKinematics::inverse(const PlanarVelocity &velocity) const
{
  const double rotation = rotation_lever_ * velocity.yaw;
  return {{(velocity.x - velocity.y - rotation) / wheel_radius_,
           (velocity.x + velocity.y + rotation) / wheel_radius_,
           (velocity.x + velocity.y - rotation) / wheel_radius_,
           (velocity.x - velocity.y + rotation) / wheel_radius_}};
}
}  // namespace hero_chassis_controller
