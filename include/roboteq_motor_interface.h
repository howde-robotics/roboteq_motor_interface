// Author(s): Kelvin Kang (kelvinkang@cmu.edu)
// This file is subject to the terms and conditions defined in the file 'LICENSE',
// which is part of this source code package

#pragma once

#include "roboteq_api/src/RoboteqDevice.h"
#include "roboteq_api/src/ErrorCodes.h"

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <string>
#include <chrono>

namespace dragoon
{
class DragoonKinematicsModel final
{
public:
  struct MotorVelocity
  {
    double left_motor_rpm;
    double right_motor_rpm;
  };

private:
};

class RoboteqMotorInterface final
{
public:
  RoboteqMotorInterface();
  ~RoboteqMotorInterface();

  void run();

private:
  RoboteqDevice roboteq_dev_;
  int motor_max_rpm;
  int rpm_to_vel_;

  // ROS params
  std::string port_;
  double cmd_vel_timeout_limit_;
  double timer_freq_;
  double slip_factor_;
  double wheel_diameter_;
  double vehicle_width_;

  // Health-related: if this exceeds a threshold, this node will send stop command to motor
  ros::Duration time_since_cmd_vel_;
  ros::Time last_cmd_vel_time_;

  // Static constants
  static constexpr int kDragoonLeftMotor = 1;
  static constexpr int kDragoonRightMotor = 2;
  // Maybe unused
  static constexpr int kMinCommand = -1000;
  static constexpr int kMaxCommand = 1000;
  static constexpr int kMaxRpm = 122;
  static constexpr int kConfigMaxRpmCh = 54;
  static constexpr int kSetRpmCh = 3;
  static constexpr int kGetRpmCh = 7;

  [[nodiscard]] bool sendCmdVelToMotors();
  [[nodiscard]] bool CheckCmdVelAge();
  void stopMotors();
  [[nodiscard]] bool readEncodersAndPubOdom();

  // ROS stuff
  ros::NodeHandle nh_, private_nh_;
  ros::Subscriber cmd_vel_sub_;
  ros::Publisher odom_pub_;
  ros::Publisher heartbeat_pub_;
  ros::Timer timer_;
  geometry_msgs::Twist curr_cmd_vel_;

  void initRos();
  void initRoboteq();
  void timerCallback(const ros::TimerEvent& e);
  void cmdVelCallback(const geometry_msgs::Twist& twistMsg);
};

}  // namespace dragoon