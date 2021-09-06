cmake_minimum_required(VERSION 3.10.2)
project(roboteq_motor_interface)
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++17 -pthread")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
)

# generate_messages(
#   DEPENDENCIES actionlib_msgs std_msgs
# )

catkin_package(CATKIN_DEPENDS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
  include
  third_party
)

add_library(skid_steer_kinematics src/skid_steer_kinematics.cpp)

add_executable(roboteq_motor_interface 
  src/roboteq_motor_interface.cpp
  third_party/roboteq_api/RoboteqDeviceARM64.o
  #third_party/roboteq_api/RoboteqDeviceX86.o
)

target_link_libraries(roboteq_motor_interface
  skid_steer_kinematics
  ${catkin_LIBRARIES}
)

catkin_add_gtest(${PROJECT_NAME}-test tests/unit_tests.cpp)
if (TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test skid_steer_kinematics)
endif()

