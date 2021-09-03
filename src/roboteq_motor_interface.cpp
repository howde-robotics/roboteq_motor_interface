// Author(s): Kelvin Kang (kelvinkang@cmu.edu)
// This file is subject to the terms and conditions defined in the file 'LICENSE',
// which is part of this source code package

#include "roboteq_motor_interface.h"
#include <signal.h>

namespace
{
volatile sig_atomic_t stop_sigint;
void inthand(int signum)
{
  stop_sigint = 1;
}
}  // namespace

namespace dragoon
{
RoboteqMotorInterface::RoboteqMotorInterface() : private_nh_("~")
{
  initRoboteq();
  initRos();

  if (roboteq_dev_.GetConfig(kConfigMaxRpmCh, motor_max_rpm) != RQ_SUCCESS)
  {
    ROS_ERROR("Failed to get max RPM from Roboteq Device. Shutting Down.");
    ros::shutdown();
  }

  static constexpr double LARGE_DURATION = 1E5;
  time_since_cmd_vel_ = ros::Duration(LARGE_DURATION);
  last_cmd_vel_time_ = ros::Time::now();
  rpm_to_vel_ = M_PI * wheel_diameter_ * slip_factor_;
  curr_cmd_vel_.angular.z = 0;
  curr_cmd_vel_.linear.x = 0;
}

void RoboteqMotorInterface::run()
{
  time_since_cmd_vel_ = ros::Time::now() - last_cmd_vel_time_;
  if (time_since_cmd_vel_.toSec() > health_timeout_limit_)
  {
    ROS_WARN("Cmd Vel to Roboteq Interface is too old t=%f, stopping motors", time_since_cmd_vel_.toSec());
    // stopMotors();
    return;
  }
}

void RoboteqMotorInterface::readAndPubEncoders()
{
  int enc_rpm_left = 0;
  int enc_rpm_right = 0;

  if (roboteq_dev_.GetValue(kGetRpmCh, kDragoonLeftMotor, enc_rpm_left) != RQ_SUCCESS)
  {
    ROS_WARN("Failed to get motor RPM from Left Motor.");
    return;
  }
  if (roboteq_dev_.GetValue(kGetRpmCh, kDragoonRightMotor, enc_rpm_right) != RQ_SUCCESS)
  {
    ROS_WARN("Failed to get motor RPM from Right Motor.");
    return;
  }

  nav_msgs::Odometry odom_msg;
  odom_msg.header.frame_id = "odom";
  odom_msg.header.stamp = ros::Time::now();
  odom_msg.twist.twist.linear.x = 0;
}

void RoboteqMotorInterface::stopMotors()
{
  roboteq_dev_.SetCommand(kSetRpmCh, kDragoonLeftMotor, 0);
  roboteq_dev_.SetCommand(kSetRpmCh, kDragoonRightMotor, 0);
}

void RoboteqMotorInterface::initRoboteq()
{
  const int status = roboteq_dev_.Connect(port_);
  if (status != RQ_SUCCESS)
  {
    ROS_ERROR("Failed to connect to Roboteq Device. Shutting Down.");
    ros::shutdown();
  }

  // while (roboteq_dev_.Connect(port_) != RQ_SUCCESS && !stop_sigint)
  // {
  //   ROS_ERROR("Failed to connect to Roboteq Device");
  //   ros::Duration(0.2).sleep();
  // }
}

void RoboteqMotorInterface::initRos()
{
  private_nh_.param<string>("port_", port_, std::string("/dev/roboteq"));
  private_nh_.param<double>("health_timeout_limit_", health_timeout_limit_, 0.05);  // secs
  private_nh_.param<double>("timetimer_freq_rFreq_", timer_freq_, 50.0);            // Hz
  private_nh_.param<double>("slip_factor_", slip_factor_, 1.0);                     // ratio
  private_nh_.param<double>("wheel_diameter_", wheel_diameter_, 0.1);               // m
  private_nh_.param<double>("vehicle_width_", vehicle_width_, 0.2);                 // m

  cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1, &RoboteqMotorInterface::cmdVelCallback, this);

  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 1);

  timer_ = nh_.createTimer(ros::Rate(timer_freq_), &RoboteqMotorInterface::timerCallback, this);
}

void RoboteqMotorInterface::cmdVelCallback(const geometry_msgs::Twist& twistMsg)
{
  const auto time_now = ros::Time::now();
  curr_cmd_vel_ = twistMsg;
  time_since_cmd_vel_ = time_now - last_cmd_vel_time_;
  last_cmd_vel_time_ = time_now;
}

void RoboteqMotorInterface::timerCallback(const ros::TimerEvent& e)
{
  run();
}

}  // namespace dragoon

int main(int argc, char** argv)
{
  // std::string response = "";
  // RoboteqDevice roboteq_dev;

  // int status = roboteq_dev.Connect("/dev/roboteq");
  // if (status != 0)
  // {
  //   ROS_ERROR("Failed to connect to Roboteq Device");
  //   return EXIT_FAILURE;
  // }

  signal(SIGINT, inthand);

  ros::init(argc, argv, "simple_node");
  dragoon::RoboteqMotorInterface node;
  while (ros::ok())
    ros::spinOnce();
  return 0;
}