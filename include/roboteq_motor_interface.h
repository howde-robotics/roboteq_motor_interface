// Author(s): Kelvin Kang (kelvinkang@cmu.edu)
// This file is subject to the terms and conditions defined in the file 'LICENSE',
// which is part of this source code package

#pragma once

#include "roboteq_api/src/RoboteqDevice.h"
#include "roboteq_api/src/ErrorCodes.h"
#include "skid_steer_kinematics.h"

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <string>
#include <algorithm>

namespace dragoon
{

class RoboteqMotorInterface final
{
public:
  RoboteqMotorInterface();
  ~RoboteqMotorInterface();

  void run();

private:
  RoboteqDevice roboteq_dev_;
  SkidSteerKinematics dragoon_kinematics_;

  // Odom related
  ros::Duration time_since_odom_pub_;
  ros::Time last_odom_pub_time_;
  double vel_x_moving_avg_ = 0.0;
  sensor_msgs::Imu curr_imu_;
  double curr_yaw_ = 0;
  double curr_x_ = 0;
  double curr_y_ = 0;

  // ROS params
  std::string port_;
  double cmd_vel_timeout_limit_;
  double timer_freq_;
  double wheel_base_;
  double slip_ratio_;
  double wheel_radius_;
  double vehicle_width_;
  int ema_num_points_;
  double ema_alpha_;

  // Health-related: if this exceeds a threshold, this node will send stop command to motor
  ros::Duration time_since_cmd_vel_;
  ros::Time last_cmd_vel_time_;

  // Static constants
  static constexpr int kDragoonLeftMotor = 2;
  static constexpr int kDragoonRightMotor = 1;
  // Maybe unused
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
  ros::Subscriber imu_sub_;
  ros::Publisher odom_pub_;
  ros::Publisher heartbeat_pub_;
  ros::Timer timer_;
  geometry_msgs::Twist curr_cmd_vel_;

  void initRos();
  void initRoboteq();
  void timerCallback(const ros::TimerEvent& e);
  void cmdVelCallback(const geometry_msgs::Twist& twistMsg);
  void imuCallback(const sensor_msgs::Imu& imuMsg);
};

}  // namespace dragoon
