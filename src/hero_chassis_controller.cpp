// Copyright 2026 taoist162-cyber
#include "hero_chassis_controller/hero_chassis_controller.h"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
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
  double radius, base, track;
  controller_nh.param("wheel_radius", radius, 0.07625);
  controller_nh.param("wheel_base", base, 0.4);
  controller_nh.param("wheel_track", track, 0.4);
  try
  {
    kinematics_ = MecanumKinematics(radius, base, track);
  }
  catch (const std::invalid_argument &exception)
  {
    ROS_ERROR_STREAM(exception.what());
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
  controller_nh.param<std::string>("odom_frame", odom_frame_, "odom");
  controller_nh.param<std::string>("base_frame", base_frame_, "base_link");
  controller_nh.param("publish_odom_tf", publish_odom_tf_, true);
  if (odom_frame_.empty() || base_frame_.empty() || odom_frame_ == base_frame_)
  {
    ROS_ERROR("Distinct, nonempty odom_frame and base_frame are required.");
    return false;
  }
  odom_publisher_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(controller_nh, "/odom", 1));
  auto &odom = odom_publisher_->msg_;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_frame_;
  // Placeholder uncertainty for planar wheel odometry; tune against ground truth.
  const std::array<double, 6> variance{{0.01, 0.01, 1e6, 1e6, 1e6, 0.05}};
  for (std::size_t i = 0; i < variance.size(); ++i)
  {
    odom.pose.covariance[6 * i + i] = variance[i];
    odom.twist.covariance[6 * i + i] = variance[i];
  }
  if (publish_odom_tf_)
  {
    tf_publisher_.reset(new realtime_tools::RealtimePublisher<tf2_msgs::TFMessage>(controller_nh, "/tf", 1));
    tf_publisher_->msg_.transforms.resize(1);
    tf_publisher_->msg_.transforms[0].header.frame_id = odom_frame_;
    tf_publisher_->msg_.transforms[0].child_frame_id = base_frame_;
  }
  command_subscriber_ = controller_nh.subscribe("/cmd_vel", 1, &HeroChassisController::commandCallback, this);
  return true;
}

void HeroChassisController::commandCallback(const geometry_msgs::TwistConstPtr &message)
{
  Command command;
  command.stamp = ros::Time::now();
  command.velocity = {message->linear.x, message->linear.y, message->angular.z};
  command.valid = std::isfinite(command.velocity.x) && std::isfinite(command.velocity.y) &&
                  std::isfinite(command.velocity.yaw);
  command_buffer_.writeFromNonRT(command);
}

void HeroChassisController::starting(const ros::Time &time)
{
  starting_time_ = time;
  last_publish_time_ = time;
  last_odom_publish_time_ = time;
  last_update_time_ = time;
  odometry_.reset();
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
  if (time < last_update_time_)
  {
    starting(time);
    return;
  }
  last_update_time_ = time;
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
  updateOdometry(time, period);
  const Command command = *command_buffer_.readFromRT();
  const double age = (time - command.stamp).toSec();
  std::array<double, 4> targets{};
  if (command.valid && command.stamp > starting_time_ && age >= 0.0 && age <= command_timeout_)
  {
    targets = kinematics_.inverse(command.velocity);
    double peak = 0.0;
    for (double speed : targets)
    {
      if (!std::isfinite(speed))
      {
        stopWheels();
        return;
      }
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

void HeroChassisController::updateOdometry(const ros::Time &time, const ros::Duration &period)
{
  std::array<double, 4> speeds;
  for (std::size_t i = 0; i < joints_.size(); ++i)
  {
    speeds[i] = joints_[i].getVelocity();
  }
  const PlanarVelocity velocity = kinematics_.forward(speeds);
  odometry_.update(velocity, period.toSec());
  if ((time - last_odom_publish_time_).toSec() < 1.0 / publish_rate_)
  {
    return;
  }
  last_odom_publish_time_ = time;
  const auto &pose = odometry_.getPose();
  geometry_msgs::Quaternion orientation;
  orientation.z = std::sin(pose.yaw / 2.0);
  orientation.w = std::cos(pose.yaw / 2.0);
  if (odom_publisher_->trylock())
  {
    auto &message = odom_publisher_->msg_;
    message.header.stamp = time;
    message.pose.pose.position.x = pose.x;
    message.pose.pose.position.y = pose.y;
    message.pose.pose.orientation = orientation;
    message.twist.twist.linear.x = velocity.x;
    message.twist.twist.linear.y = velocity.y;
    message.twist.twist.angular.z = velocity.yaw;
    odom_publisher_->unlockAndPublish();
  }
  if (tf_publisher_ && tf_publisher_->trylock())
  {
    auto &transform = tf_publisher_->msg_.transforms[0];
    transform.header.stamp = time;
    transform.transform.translation.x = pose.x;
    transform.transform.translation.y = pose.y;
    transform.transform.rotation = orientation;
    tf_publisher_->unlockAndPublish();
  }
}

void HeroChassisController::stopping(const ros::Time &)
{
  stopWheels();
}
}  // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
