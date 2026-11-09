// Copyright 2026 taoist162-cyber
#include "hero_chassis_controller/hero_chassis_controller.h"

#include <algorithm>
#include <cmath>
#include <set>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

namespace hero_chassis_controller
{
bool HeroChassisController::init(hardware_interface::EffortJointInterface *hardware, ros::NodeHandle &,
                                 ros::NodeHandle &controller_nh)
{
  std::vector<std::string> names;
  if (!hardware || !controller_nh.getParam("joints", names) || names.size() != 4 ||
      std::set<std::string>(names.begin(), names.end()).size() != 4)
  {
    ROS_ERROR("Expected four distinct effort joints in FL, FR, RL, RR order.");
    return false;
  }
  controller_nh.param("command_timeout", command_timeout_, 0.5);
  controller_nh.param("max_wheel_speed", max_wheel_speed_, 50.0);
  controller_nh.param("max_effort", max_effort_, 5.0);
  controller_nh.param("publish_rate", publish_rate_, 50.0);
  for (double value : {command_timeout_, max_wheel_speed_, max_effort_, publish_rate_})
  {
    if (!std::isfinite(value) || value <= 0.0)
    {
      ROS_ERROR("Timeout, speed/effort limits and publish rate must be finite and positive.");
      return false;
    }
  }
  if (max_effort_ > 5.0 || max_wheel_speed_ > 50.0)
  {
    ROS_ERROR("Limits exceed Hero URDF ratings: 5 Nm and 50 rad/s.");
    return false;
  }
  try
  {
    for (std::size_t i = 0; i < joints_.size(); ++i)
    {
      joints_[i] = hardware->getHandle(names[i]);
      ros::NodeHandle pid_nh(controller_nh, "gains/" + names[i]);
      for (const std::string key : {"p", "i", "d", "i_clamp"})
      {
        double value;
        if (!pid_nh.getParam(key, value) || !std::isfinite(value) || value < 0.0)
        {
          ROS_ERROR_STREAM("Missing or invalid PID parameter: " << pid_nh.resolveName(key));
          return false;
        }
      }
      // init(NodeHandle) also creates a dynamic_reconfigure server for this wheel.
      if (!pids_[i].init(pid_nh))
      {
        return false;
      }
      state_publishers_[i].reset(new StatePublisher(controller_nh, names[i] + "/state", 1));
    }
  }
  catch (const hardware_interface::HardwareInterfaceException &exception)
  {
    ROS_ERROR_STREAM(exception.what());
    return false;
  }
  command_buffer_.initRT(Command{});
  command_subscriber_ = controller_nh.subscribe("wheel_commands", 1, &HeroChassisController::commandCallback, this);
  return true;
}

void HeroChassisController::commandCallback(const std_msgs::Float64MultiArrayConstPtr &message)
{
  Command command;
  command.stamp = ros::Time::now();
  command.valid = message->data.size() == 4;
  if (command.valid)
  {
    for (std::size_t i = 0; i < command.wheels.size(); ++i)
    {
      command.wheels[i] = message->data[i];
      command.valid = command.valid && std::isfinite(command.wheels[i]);
    }
  }
  command_buffer_.writeFromNonRT(command);
}

void HeroChassisController::starting(const ros::Time &time)
{
  starting_time_ = time;
  last_publish_time_ = time;
  stopWheels();
}

void HeroChassisController::stopWheels()
{
  for (std::size_t i = 0; i < joints_.size(); ++i)
  {
    joints_[i].setCommand(0.0);
    pids_[i].reset();
  }
}

void HeroChassisController::update(const ros::Time &time, const ros::Duration &period)
{
  if (!std::isfinite(period.toSec()) || period.toSec() <= 0.0)
  {
    stopWheels();
    return;
  }
  for (const auto &joint : joints_)
  {
    if (!std::isfinite(joint.getVelocity()))
    {
      stopWheels();
      return;
    }
  }
  const Command command = *command_buffer_.readFromRT();
  const double age = (time - command.stamp).toSec();
  std::array<double, 4> targets{};
  if (command.valid && command.stamp > starting_time_ && age >= 0.0 && age <= command_timeout_)
  {
    targets = command.wheels;
    double peak = 0.0;
    for (double speed : targets)
    {
      peak = std::max(peak, std::abs(speed));
    }
    if (peak > max_wheel_speed_)
    {
      for (double &speed : targets)
      {
        speed *= max_wheel_speed_ / peak;
      }
    }
  }
  for (std::size_t i = 0; i < joints_.size(); ++i)
  {
    const double effort = pids_[i].computeCommand(targets[i] - joints_[i].getVelocity(), period);
    if (!std::isfinite(effort))
    {
      stopWheels();
      return;
    }
    joints_[i].setCommand(std::max(-max_effort_, std::min(max_effort_, effort)));
  }
  publishWheelStates(time, period, targets);
}

void HeroChassisController::publishWheelStates(const ros::Time &time, const ros::Duration &period,
                                               const std::array<double, 4> &targets)
{
  if ((time - last_publish_time_).toSec() < 1.0 / publish_rate_)
  {
    return;
  }
  last_publish_time_ = time;
  for (std::size_t i = 0; i < joints_.size(); ++i)
  {
    auto &publisher = state_publishers_[i];
    if (publisher->trylock())
    {
      auto &message = publisher->msg_;
      message.header.stamp = time;
      message.set_point = targets[i];
      message.process_value = joints_[i].getVelocity();
      message.error = targets[i] - message.process_value;
      message.time_step = period.toSec();
      message.command = joints_[i].getCommand();
      bool antiwindup;
      double i_min;
      pids_[i].getGains(message.p, message.i, message.d, message.i_clamp, i_min, antiwindup);
      message.antiwindup = antiwindup;
      publisher->unlockAndPublish();
    }
  }
}

void HeroChassisController::stopping(const ros::Time &)
{
  stopWheels();
}
}  // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
