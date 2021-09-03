#include <gtest/gtest.h>
#include "skid_steer_kinematics.h"

/*
 * Testing the forward and inverse of the kinematics to verify the math is correct
 */

TEST(KinematicsTest, ForwardInverseSimple)
{
	float vehicle_width = 1.0;
	float wheel_rad = 1.0;
	float slip_ratio = 1.0;
	float wheel_base = 1.0;

	dragoon::SkidSteerKinematics dragoon{
		vehicle_width,
		wheel_rad,
		slip_ratio,
		wheel_base
	};

	dragoon::SkidSteerKinematics::BodyVelocities bodyIn {1.0, 1.0};

	dragoon::SkidSteerKinematics::WheelVelocities compare {0.5, 1.5};
	dragoon::SkidSteerKinematics::WheelVelocities wheelTest = dragoon.calcWheelVelFromBodyVel(bodyIn);

	ASSERT_DOUBLE_EQ(compare.left_wheel_vel_x, wheelTest.left_wheel_vel_x);
	ASSERT_DOUBLE_EQ(compare.right_wheel_vel_x, wheelTest.right_wheel_vel_x);

	// and try the inverse
	dragoon::SkidSteerKinematics::BodyVelocities bodyTest = dragoon.calcBodyVelFromWheelVel(wheelTest);

	ASSERT_DOUBLE_EQ(bodyIn.linear_x, bodyTest.linear_x);
	ASSERT_DOUBLE_EQ(bodyIn.angular_z, bodyTest.angular_z);
}

TEST(KinematicsTest, ForwardInverseForwardDrive)
{
	float vehicle_width = 2.532;
	float wheel_rad = 0.232;
	float slip_ratio = 1.0;
	float wheel_base = 0.25;

	dragoon::SkidSteerKinematics dragoon{
		vehicle_width,
		wheel_rad,
		slip_ratio,
		wheel_base
	};

	// drive forward
	dragoon::SkidSteerKinematics::BodyVelocities bodyIn {1.0, 0.0};
	// divide by wheel radius to get the output angular velocity
	dragoon::SkidSteerKinematics::WheelVelocities compare {1.0, 1.0};

	dragoon::SkidSteerKinematics::WheelVelocities wheelTest = dragoon.calcWheelVelFromBodyVel(bodyIn);

	ASSERT_DOUBLE_EQ(compare.left_wheel_vel_x, wheelTest.left_wheel_vel_x);
	ASSERT_DOUBLE_EQ(compare.right_wheel_vel_x, wheelTest.right_wheel_vel_x);

	// and try the inverse
	dragoon::SkidSteerKinematics::BodyVelocities bodyTest = dragoon.calcBodyVelFromWheelVel(wheelTest);

	ASSERT_DOUBLE_EQ(bodyIn.linear_x, bodyTest.linear_x);
	ASSERT_DOUBLE_EQ(bodyIn.angular_z, bodyTest.angular_z);
}

TEST(KinematicsTest, ForwardInverseReverseDrive)
{
	float vehicle_width = 2.532;
	float wheel_rad = 0.232;
	float slip_ratio = 1.0;
	float wheel_base = 0.25;

	dragoon::SkidSteerKinematics dragoon{
		vehicle_width,
		wheel_rad,
		slip_ratio,
		wheel_base
	};

	// drive forward
	dragoon::SkidSteerKinematics::BodyVelocities bodyIn {-1.0, 0.0};
	// divide by wheel radius to get the output angular velocity
	dragoon::SkidSteerKinematics::WheelVelocities compare {-1.0, -1.0};

	dragoon::SkidSteerKinematics::WheelVelocities wheelTest = dragoon.calcWheelVelFromBodyVel(bodyIn);

	ASSERT_DOUBLE_EQ(compare.left_wheel_vel_x, wheelTest.left_wheel_vel_x);
	ASSERT_DOUBLE_EQ(compare.right_wheel_vel_x, wheelTest.right_wheel_vel_x);

	// and try the inverse
	dragoon::SkidSteerKinematics::BodyVelocities bodyTest = dragoon.calcBodyVelFromWheelVel(wheelTest);

	ASSERT_DOUBLE_EQ(bodyIn.linear_x, bodyTest.linear_x);
	ASSERT_DOUBLE_EQ(bodyIn.angular_z, bodyTest.angular_z);
}

TEST(KinematicsTest, ForwardInverseRotation)
{
	float vehicle_width = 2.532;
	float wheel_rad = 0.232;
	float slip_ratio = 1.0;
	float wheel_base = 0.25;

	dragoon::SkidSteerKinematics dragoon{
		vehicle_width,
		wheel_rad,
		slip_ratio,
		wheel_base
	};

	// drive forward
	dragoon::SkidSteerKinematics::BodyVelocities bodyIn {0.0, 1.0};
	// divide by wheel radius to get the output angular velocity
	dragoon::SkidSteerKinematics::WheelVelocities compare {-0.125, 0.125};

	dragoon::SkidSteerKinematics::WheelVelocities wheelTest = dragoon.calcWheelVelFromBodyVel(bodyIn);

	ASSERT_DOUBLE_EQ(compare.left_wheel_vel_x, wheelTest.left_wheel_vel_x);
	ASSERT_DOUBLE_EQ(compare.right_wheel_vel_x, wheelTest.right_wheel_vel_x);

	// and try the inverse
	dragoon::SkidSteerKinematics::BodyVelocities bodyTest = dragoon.calcBodyVelFromWheelVel(wheelTest);

	ASSERT_DOUBLE_EQ(bodyIn.linear_x, bodyTest.linear_x);
	ASSERT_DOUBLE_EQ(bodyIn.angular_z, bodyTest.angular_z);
}

int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}