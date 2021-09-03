// Author(s): Kelvin Kang (kelvinkang@cmu.edu)
// This file is subject to the terms and conditions defined in the file 'LICENSE',
// which is part of this source code package

#include "skid_steer_kinematics.h"
#include <math.h>

namespace dragoon
{
SkidSteerKinematics::SkidSteerKinematics(double wheel_base, double vehicle_width, double wheel_radius,
                                         double slip_ratio)
  : vehicle_width_(vehicle_width), wheel_radius_(wheel_radius), slip_ratio_(slip_ratio), wheel_base_(wheel_base)
{
}

SkidSteerKinematics::MotorVelocities SkidSteerKinematics::calcMotorVelFromBodyVel(const BodyVelocities& body_vel) const
{
  MotorVelocities out;
  out = calcMotorVelFromWheelVel(calcWheelVelFromBodyVel(body_vel));
  return out;
}

SkidSteerKinematics::BodyVelocities SkidSteerKinematics::calcBodyVelFromMotorVel(const MotorVelocities& motor_vel) const
{
  BodyVelocities out;
  out = calcBodyVelFromWheelVel(calcWheelVelFromMotorVel(motor_vel));
  return out;
}

SkidSteerKinematics::BodyVelocities SkidSteerKinematics::calcBodyVelFromWheelVel(const WheelVelocities& wheel_vel) const
{
  BodyVelocities out;
  out.linear_x = (wheel_vel.left_wheel_vel_x + wheel_vel.right_wheel_vel_x) / 2.0;
  out.angular_z = (wheel_vel.right_wheel_vel_x - wheel_vel.left_wheel_vel_x) / vehicle_width_;
  return out;
}

SkidSteerKinematics::WheelVelocities SkidSteerKinematics::calcWheelVelFromBodyVel(const BodyVelocities& body_vel) const
{
  WheelVelocities out;
  out.left_wheel_vel_x = (body_vel.linear_x - body_vel.angular_z * wheel_base_ / 2.0) / wheel_radius_;
  out.right_wheel_vel_x = (body_vel.linear_x + body_vel.angular_z * wheel_base_ / 2.0) / wheel_radius_;
  return out;
}

SkidSteerKinematics::MotorVelocities
SkidSteerKinematics::calcMotorVelFromWheelVel(const WheelVelocities& wheel_vel) const
{
  MotorVelocities out;

  // return motor_vel = (wheel_vel / wheel_radius) * (1 + slip_ratio)
  const auto op = [this](const double wheel_vel) {
    return this->RadPerSecToRpm(wheel_vel / this->getWheelRadius() * (1.0 + this->getSlipRatio()));
  };

  out.left_motor_rpm = op(wheel_vel.left_wheel_vel_x);
  out.right_motor_rpm = op(wheel_vel.right_wheel_vel_x);
  return out;
}

SkidSteerKinematics::WheelVelocities
SkidSteerKinematics::calcWheelVelFromMotorVel(const MotorVelocities& motor_vel) const
{
  WheelVelocities out;

  // return wheel_vel = (motor_vel * wheel_radius) / (1 + slip_ratio)
  const auto op = [this](const double motor_vel) -> double {
    return this->RpmToRadPerSec(motor_vel) * this->getWheelRadius() / (1.0 + this->getSlipRatio());
  };

  out.left_wheel_vel_x = op(motor_vel.left_motor_rpm);
  out.right_wheel_vel_x = op(motor_vel.right_motor_rpm);
  return out;
}

double SkidSteerKinematics::RpmToRadPerSec(const double rpm)
{
  return rpm * 2 * M_PI / 60.0;
}
double SkidSteerKinematics::RadPerSecToRpm(const double rad_per_sec)
{
  return rad_per_sec * 60.0 / (2 * M_PI);
}

}  // namespace dragoon