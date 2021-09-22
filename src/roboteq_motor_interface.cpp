// Author(s): Kelvin Kang (kelvinkang@cmu.edu)
// This file is subject to the terms and conditions defined in the file 'LICENSE',
// which is part of this source code package

#include <tf/tf.h>
#include <algorithm>
#include "roboteq_motor_interface.h"

namespace dragoon
{
RoboteqMotorInterface::RoboteqMotorInterface() : private_nh_("~")
{
  this->initRos();
  this->initRoboteq();

  dragoon_kinematics_.setWheelBase(wheel_base_);
  dragoon_kinematics_.setVehicleWidth(vehicle_width_);
  dragoon_kinematics_.setWheelRadius(wheel_radius_);
  dragoon_kinematics_.setSlipRatio(slip_ratio_);

  ema_alpha_ = 2.0 / (1.0 + ema_num_points_);

  static constexpr double LARGE_DURATION = 1E5;
  time_since_cmd_vel_ = ros::Duration(LARGE_DURATION);
  last_cmd_vel_time_ = ros::Time::now();
  last_odom_pub_time_ = ros::Time::now();
  curr_cmd_vel_.angular.z = 0;
  curr_cmd_vel_.linear.x = 0;
  time_since_odom_pub_.nsec = 0;
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
    ROS_WARN("Cmd Vel to Roboteq Interface is too old t=%.2fs, stopping motors", time_since_cmd_vel_.toSec());
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
  int enc_rpm_rel_left = 0;
  int enc_rpm_rel_right = 0;

  if (roboteq_dev_.GetValue(kGetRpmCh, kDragoonLeftMotor, enc_rpm_rel_left) != RQ_SUCCESS)
  {
    ROS_WARN("Failed to get motor RPM from Left Motor.");
    return false;
  }
  if (roboteq_dev_.GetValue(kGetRpmCh, kDragoonRightMotor, enc_rpm_rel_right) != RQ_SUCCESS)
  {
    ROS_WARN("Failed to get motor RPM from Right Motor.");
    return false;
  }

  SkidSteerKinematics::MotorVelocities motor_vel;
  motor_vel.left_motor_rpm = enc_rpm_rel_left * kMaxRpm / kMaxCommand;
  motor_vel.right_motor_rpm = enc_rpm_rel_right * kMaxRpm / kMaxCommand;
  SkidSteerKinematics::BodyVelocities body_vel;
  body_vel = dragoon_kinematics_.calcBodyVelFromMotorVel(motor_vel);

  vel_x_moving_avg_ = (ema_alpha_ * body_vel.linear_x) + (1.0 - ema_alpha_) * vel_x_moving_avg_;

  time_since_odom_pub_ = ros::Time::now() - last_odom_pub_time_;
  curr_yaw_ += curr_imu_.angular_velocity.z * time_since_odom_pub_.toSec();
  curr_x_ += vel_x_moving_avg_ * time_since_odom_pub_.toSec() * std::cos(curr_yaw_);
  curr_y_ += vel_x_moving_avg_ * time_since_odom_pub_.toSec() * std::sin(curr_yaw_);

  nav_msgs::Odometry odom_msg;
  odom_msg.header.frame_id = "odom";
  odom_msg.header.stamp = ros::Time::now();
  odom_msg.pose.pose.position.x = curr_x_;
  odom_msg.pose.pose.position.y = curr_y_;
  odom_msg.pose.pose.orientation = (geometry_msgs::Quaternion)tf::createQuaternionMsgFromYaw(curr_yaw_);
  odom_msg.twist.twist.linear.x = vel_x_moving_avg_;
  odom_msg.twist.twist.angular.z = curr_imu_.angular_velocity.z;
  odom_pub_.publish(odom_msg);

  last_odom_pub_time_ = ros::Time::now();

  return true;
};

[[nodiscard]] bool RoboteqMotorInterface::CheckCmdVelAge() {
  time_since_cmd_vel_ = ros::Time::now() - last_cmd_vel_time_;
  return time_since_cmd_vel_.toSec() < cmd_vel_timeout_limit_;
}

    [[nodiscard]] bool RoboteqMotorInterface::sendCmdVelToMotors()
{
  SkidSteerKinematics::BodyVelocities body_vel;
  body_vel.linear_x = curr_cmd_vel_.linear.x;
  body_vel.angular_z = curr_cmd_vel_.angular.z;
  SkidSteerKinematics::MotorVelocities motor_vel;
  motor_vel = dragoon_kinematics_.calcMotorVelFromBodyVel(body_vel);

  const int left_rpm_cmd = std::clamp((int)(motor_vel.left_motor_rpm), -kMaxRpm, kMaxRpm);
  const int right_rpm_cmd = std::clamp((int)(motor_vel.right_motor_rpm), -kMaxRpm, kMaxRpm);

  if (roboteq_dev_.SetCommand(kSetRpmCh, kDragoonLeftMotor, left_rpm_cmd) != RQ_SUCCESS)
  {
    ROS_WARN("Failed to send cmd to Roboteq Left Motor.");
    return false;
  }
  if (roboteq_dev_.SetCommand(kSetRpmCh, kDragoonRightMotor, right_rpm_cmd) != RQ_SUCCESS)
  {
    ROS_WARN("Failed to send cmd to Roboteq Right Motor.");
    return false;
  }

  ROS_DEBUG("Roboteq Motor Interface: sending cmd to left: %d rpm, right: %d rpm", left_rpm_cmd, right_rpm_cmd);
  return true;
}

void RoboteqMotorInterface::stopMotors()
{
  // ros::Duration(0.01).sleep();
  if (roboteq_dev_.SetCommand(kSetRpmCh, kDragoonLeftMotor, 0) != RQ_SUCCESS)
  {
    ROS_ERROR("Failed to stop Roboteq Left Motor. Disconnecting.");
    // TODO (kelvinkang): decide if we want to disconnect when failed to stop motors
    // roboteq_dev_.Disconnect();
    // ros::shutdown();
    // return;
  }
  // ros::Duration(0.01).sleep();
  if (roboteq_dev_.SetCommand(kSetRpmCh, kDragoonRightMotor, 0) != RQ_SUCCESS)
  {
    ROS_ERROR("Failed to stop Roboteq Right Motor. Disconnecting.");
    // roboteq_dev_.Disconnect();
    // ros::shutdown();
    // return;
  }
}

void RoboteqMotorInterface::initRoboteq()
{
  if (roboteq_dev_.Connect(port_) != RQ_SUCCESS)
  {
    ROS_ERROR("Failed to connect to Roboteq Device. Shutting Down.");
    ros::shutdown();
  }

  if (roboteq_dev_.SetConfig(kConfigMaxRpmCh, kDragoonLeftMotor, kMaxRpm) != RQ_SUCCESS)
  {
    ROS_ERROR("Failed to set max RPM on Left Motor Shutting Down.");
    ros::shutdown();
  }
  if (roboteq_dev_.SetConfig(kConfigMaxRpmCh, kDragoonRightMotor, kMaxRpm) != RQ_SUCCESS)
  {
    ROS_ERROR("Failed to set max RPM on Left Motor Shutting Down.");
    ros::shutdown();
  }
}

RoboteqMotorInterface::~RoboteqMotorInterface()
{
  this->stopMotors();
  roboteq_dev_.Disconnect();
}

void RoboteqMotorInterface::initRos()
{
  private_nh_.param<string>("port_", port_, std::string("/dev/roboteq"));
  private_nh_.param<double>("cmd_vel_timeout_limit_", cmd_vel_timeout_limit_, 0.2);  // secs
  private_nh_.param<double>("timer_freq_", timer_freq_, 50.0);                       // Hz
  private_nh_.param<double>("slip_ratio_", slip_ratio_, 0.1);                        // ratio
  private_nh_.param<double>("wheel_base_", wheel_base_, 0.45);                       // m
  private_nh_.param<double>("wheel_radius_", wheel_radius_, 0.07);                   // m
  private_nh_.param<double>("vehicle_width_", vehicle_width_, 0.25);                 // m
  private_nh_.param<int>("ema_num_points_", ema_num_points_, 10);                    // count

  cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1, &RoboteqMotorInterface::cmdVelCallback, this);
  imu_sub_ = nh_.subscribe("imu", 1, &RoboteqMotorInterface::imuCallback, this);

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

void RoboteqMotorInterface::imuCallback(const sensor_msgs::Imu& imuMsg)
{
  curr_imu_ = imuMsg;
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
