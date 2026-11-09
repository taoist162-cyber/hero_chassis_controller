// Copyright 2026 taoist162-cyber
#ifndef HERO_CHASSIS_CONTROLLER_MECANUM_KINEMATICS_H
#define HERO_CHASSIS_CONTROLLER_MECANUM_KINEMATICS_H

#include <array>

namespace hero_chassis_controller
{
struct PlanarVelocity
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

class MecanumKinematics
{
public:
  MecanumKinematics(double wheel_radius = 0.07625, double wheel_base = 0.4, double wheel_track = 0.4);
  // base_link: +X forward, +Y left, +Z up. Joint axes are all +Y in the Hero URDF.
  // Joint order: front left, front right, rear left, rear right; output unit: rad/s.
  std::array<double, 4> inverse(const PlanarVelocity &velocity) const;
  PlanarVelocity forward(const std::array<double, 4> &wheel_speeds) const;

private:
  double wheel_radius_;
  double rotation_lever_;
};

struct PlanarPose
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

class WheelOdometry
{
public:
  void reset();
  void update(const PlanarVelocity &body_velocity, double dt);
  const PlanarPose &getPose() const;

private:
  PlanarPose pose_;
};
}  // namespace hero_chassis_controller

#endif  // HERO_CHASSIS_CONTROLLER_MECANUM_KINEMATICS_H
