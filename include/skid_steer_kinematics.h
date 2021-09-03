// Author(s): Kelvin Kang (kelvinkang@cmu.edu)
// This file is subject to the terms and conditions defined in the file 'LICENSE',
// which is part of this source code package

#pragma once

namespace dragoon
{
// Calculates Skid Steer Kinematics between body, wheel, and motor velocities
class SkidSteerKinematics final
{
public:
  SkidSteerKinematics() {};
  explicit SkidSteerKinematics(double wheel_base, double vehicle_width, double wheel_radius, double slip_ratio);

  struct MotorVelocities final
  {
    double left_motor_rpm;
    double right_motor_rpm;
  };

  struct WheelVelocities final
  {
    double left_wheel_vel_x;
    double right_wheel_vel_x;
  };

  struct BodyVelocities final
  {
    double linear_x;
    double angular_z;
  };

  MotorVelocities calcMotorVelFromBodyVel(const BodyVelocities& body_vel) const;
  MotorVelocities calcMotorVelFromWheelVel(const WheelVelocities& wheel_vel) const;
  BodyVelocities calcBodyVelFromMotorVel(const MotorVelocities& motor_vel) const;
  BodyVelocities calcBodyVelFromWheelVel(const WheelVelocities& wheel_vel) const;
  WheelVelocities calcWheelVelFromBodyVel(const BodyVelocities& body_vel) const;
  WheelVelocities calcWheelVelFromMotorVel(const MotorVelocities& motor_vel) const;

  double getWheelBase() const
  {
    return wheel_base_;
  }
  double getVehicleWidth() const
  {
    return vehicle_width_;
  }
  double getWheelRadius() const
  {
    return wheel_radius_;
  }
  double getSlipRatio() const
  {
    return slip_ratio_;
  }
  void setWheelBase(const double wheel_base)
  {
    wheel_base_ = wheel_base;
  }
  void setVehicleWidth(const double vehicle_width)
  {
    vehicle_width_ = vehicle_width;
  }
  void setWheelRadius(const double wheel_radius)
  {
    wheel_radius_ = wheel_radius;
  }
  void setSlipRatio(const double slip_ratio)
  {
    slip_ratio_ = slip_ratio;
  }

private:
  // length between centre of front and back wheels (m)
  double wheel_base_ = 0.8;
  // length between centre of left and right wheels (m)
  double vehicle_width_ = 0.3;
  double wheel_radius_ = 0.1;
  // slip ratio = ((wheel_ang_vel * wheel_radius) / body_vel - 1)
  double slip_ratio_ = 0.1;

  static inline double RpmToRadPerSec(const double rpm);
  static inline double RadPerSecToRpm(const double rad_per_sec);
};

}  // namespace dragoon