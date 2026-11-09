// Copyright 2026 taoist162-cyber
#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H

#include <array>
#include <memory>
#include <string>

#include <control_msgs/JointControllerState.h>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2_msgs/TFMessage.h>

#include "hero_chassis_controller/mecanum_kinematics.h"

namespace hero_chassis_controller
{
class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  bool init(hardware_interface::EffortJointInterface *hardware, ros::NodeHandle &root_nh,
            ros::NodeHandle &controller_nh) override;
  void starting(const ros::Time &time) override;
  void update(const ros::Time &time, const ros::Duration &period) override;
  void stopping(const ros::Time &time) override;

private:
  struct Command
  {
    PlanarVelocity velocity;
    ros::Time stamp;
    bool valid = false;
  };
  using StatePublisher = realtime_tools::RealtimePublisher<control_msgs::JointControllerState>;
  void commandCallback(const geometry_msgs::TwistConstPtr &message);
  void stopWheels();
  void publishWheelStates(const ros::Time &time, const ros::Duration &period,
                          const std::array<double, 4> &targets);
  void updateOdometry(const ros::Time &time, const ros::Duration &period);

  // Order throughout the package: front left, front right, rear left, rear right.
  std::array<hardware_interface::JointHandle, 4> joints_;
  std::array<control_toolbox::Pid, 4> pids_;
  std::array<std::unique_ptr<StatePublisher>, 4> state_publishers_;
  realtime_tools::RealtimeBuffer<Command> command_buffer_;
  ros::Subscriber command_subscriber_;
  MecanumKinematics kinematics_;
  WheelOdometry odometry_;
  std::unique_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odom_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<tf2_msgs::TFMessage>> tf_publisher_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_odom_tf_ = true;
  ros::Time last_odom_publish_time_;
  ros::Time last_update_time_;
  ros::Time starting_time_;
  ros::Time last_publish_time_;
  double command_timeout_ = 0.5;
  double max_wheel_speed_ = 50.0;
  double max_effort_ = 5.0;
  double publish_rate_ = 50.0;
};
}  // namespace hero_chassis_controller

#endif  // HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
