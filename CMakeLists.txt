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
  include
  third_party
)

# add_library(roboteq_api third_party/roboteq_api/src/RoboteqDevice.cpp)

add_executable(roboteq_motor_interface 
  src/roboteq_motor_interface.cpp
  # third_party/roboteq_api/src/RoboteqDevice.cpp
  third_party/roboteq_api/RoboteqDevice.o
)

target_link_libraries(roboteq_motor_interface
  ${catkin_LIBRARIES}
)
