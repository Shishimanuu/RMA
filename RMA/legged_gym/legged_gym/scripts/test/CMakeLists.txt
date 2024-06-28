cmake_minimum_required(VERSION 3.10)

project(shvan_plugins VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  shvan_msgs
  # Add other required components
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(IK_motor_command src/inverse_kinematics.cpp)
target_link_libraries(IK_motor_command ${catkin_LIBRARIES})