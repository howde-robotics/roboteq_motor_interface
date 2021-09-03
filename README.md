# Roboteq Motor Interface

Contains:
1. `RoboteqMotorInterface` class that is a ROS node takes in cmd from navigation and sends cmd to the motors with serial port. And reads motors serial encoders data and publish odom ros topic
2. `SkidSteerKinematics` class with API to calculate between body, wheel, and motor velocities of Dragoon's skid steering