// Author(s): Kelvin Kang (kelvinkang@cmu.edu)
// This file is subject to the terms and conditions defined in the file 'LICENSE',
// which is part of this source code package

#include "roboteq_motor_interface.h"

namespace dragoon
{
RoboteqMotorInterface::RoboteqMotorInterface() : private_nh_("~")
{
  this->initRoboteq();
  this->initRos();

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
  bool good_health = true;

  if (!this->readEncodersAndPubOdom())
  {
    ROS_WARN("Failed to read Roboteq encoders and Pub Odom. Stopping publishing good health.");
    good_health = false;
  }

  if (!this->CheckCmdVelAge())
  {
    ROS_ERROR("Cmd Vel to Roboteq Interface is too old t=%f, stopping motors", time_since_cmd_vel_.toSec());
    stopMotors();
    return;
  }

  if (!this->sendCmdVelToMotors())
  {
    ROS_WARN("Failed to send cmd_vel to Roboteq Motors. Stopping publishing good health.");
    good_health = false;
  }

  if (good_health)
  {
    heartbeat_pub_.publish(std_msgs::Empty());
  }
}

[[nodiscard]] bool RoboteqMotorInterface::readEncodersAndPubOdom() {
  int enc_rpm_left = 0;
  int enc_rpm_right = 0;

  if (roboteq_dev_.GetValue(kGetRpmCh, kDragoonLeftMotor, enc_rpm_left) != RQ_SUCCESS)
  {
    ROS_WARN("Failed to get motor RPM from Left Motor.");
    return false;
  }
  if (roboteq_dev_.GetValue(kGetRpmCh, kDragoonRightMotor, enc_rpm_right) != RQ_SUCCESS)
  {
    ROS_WARN("Failed to get motor RPM from Right Motor.");
    return false;
  }

  nav_msgs::Odometry odom_msg;
  odom_msg.header.frame_id = "odom";
  odom_msg.header.stamp = ros::Time::now();
  odom_msg.twist.twist.linear.x = 0;
  // TODO: figure out kinematics to know what to send
  odom_pub_.publish(odom_msg);
  return true;
};

[[nodiscard]] bool RoboteqMotorInterface::CheckCmdVelAge() {
  time_since_cmd_vel_ = ros::Time::now() - last_cmd_vel_time_;
  return time_since_cmd_vel_.toSec() > cmd_vel_timeout_limit_;
}

    [[nodiscard]] bool RoboteqMotorInterface::sendCmdVelToMotors()
{
  const double left_rpm_cmd = 0;
  const double right_rpm_cmd = 0;

  if (roboteq_dev_.SetCommand(kSetRpmCh, kDragoonLeftMotor, 0) != RQ_SUCCESS)
  {
    ROS_WARN("Failed to send cmd to Roboteq Left Motor.");
    return false;
  }
  if (roboteq_dev_.SetCommand(kSetRpmCh, kDragoonLeftMotor, 0) != RQ_SUCCESS)
  {
    ROS_WARN("Failed to send cmd to Roboteq Right Motor.");
    return false;
  }
  return true;
}

void RoboteqMotorInterface::stopMotors()
{
  if (roboteq_dev_.SetCommand(kSetRpmCh, kDragoonLeftMotor, 0) != RQ_SUCCESS)
  {
    ROS_ERROR("Failed to stop Roboteq Left Motor. Disconnecting.");
    roboteq_dev_.Disconnect();
    ros::shutdown();
    return;
  }
  if (roboteq_dev_.SetCommand(kSetRpmCh, kDragoonRightMotor, 0) != RQ_SUCCESS)
  {
    ROS_ERROR("Failed to stop Roboteq Right Motor. Disconnecting.");
    roboteq_dev_.Disconnect();
    ros::shutdown();
    return;
  }
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

RoboteqMotorInterface::~RoboteqMotorInterface()
{
  roboteq_dev_.Disconnect();
}

void RoboteqMotorInterface::initRos()
{
  private_nh_.param<string>("port_", port_, std::string("/dev/roboteq"));
  private_nh_.param<double>("cmd_vel_timeout_limit_", cmd_vel_timeout_limit_, 0.05);  // secs
  private_nh_.param<double>("timetimer_freq_rFreq_", timer_freq_, 50.0);              // Hz
  private_nh_.param<double>("slip_factor_", slip_factor_, 1.0);                       // ratio
  private_nh_.param<double>("wheel_diameter_", wheel_diameter_, 0.1);                 // m
  private_nh_.param<double>("vehicle_width_", vehicle_width_, 0.2);                   // m

  cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1, &RoboteqMotorInterface::cmdVelCallback, this);

  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 1);
  heartbeat_pub_ = nh_.advertise<std_msgs::Empty>("/roboteq_motor_interface_heartbeat", 1);

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
  this->run();
}

}  // namespace dragoon

int main(int argc, char** argv)
{
  ros::init(argc, argv, "roboteq_motor_interface");
  dragoon::RoboteqMotorInterface node;
  while (ros::ok())
    ros::spinOnce();
  return 0;
}