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
PlanarVelocity MecanumKinematics::forward(const std::array<double, 4> &wheel_speeds) const
{
  const double scale = wheel_radius_ / 4.0;
  return {scale * (wheel_speeds[0] + wheel_speeds[1] + wheel_speeds[2] + wheel_speeds[3]),
          scale * (-wheel_speeds[0] + wheel_speeds[1] + wheel_speeds[2] - wheel_speeds[3]),
          scale * (-wheel_speeds[0] + wheel_speeds[1] - wheel_speeds[2] + wheel_speeds[3]) / rotation_lever_};
}

void WheelOdometry::reset()
{
  pose_ = PlanarPose{};
}

void WheelOdometry::update(const PlanarVelocity &body_velocity, double dt)
{
  if (!std::isfinite(dt) || dt <= 0.0 || !std::isfinite(body_velocity.x) ||
      !std::isfinite(body_velocity.y) || !std::isfinite(body_velocity.yaw))
  {
    return;
  }
  // Exact SE(2) integration for constant body velocity during this control period.
  const double angle = body_velocity.yaw * dt;
  const double sinc = std::abs(angle) < 1e-6 ? 1.0 - angle * angle / 6.0 : std::sin(angle) / angle;
  const double cosc = std::abs(angle) < 1e-6 ? angle / 2.0 : (1.0 - std::cos(angle)) / angle;
  const double dx = dt * (sinc * body_velocity.x - cosc * body_velocity.y);
  const double dy = dt * (cosc * body_velocity.x + sinc * body_velocity.y);
  pose_.x += std::cos(pose_.yaw) * dx - std::sin(pose_.yaw) * dy;
  pose_.y += std::sin(pose_.yaw) * dx + std::cos(pose_.yaw) * dy;
  pose_.yaw = std::atan2(std::sin(pose_.yaw + angle), std::cos(pose_.yaw + angle));
}

const PlanarPose &WheelOdometry::getPose() const
{
  return pose_;
}
}  // namespace hero_chassis_controller
