cmake_minimum_required(VERSION 3.10)

project(shvan_plugins VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED NO_MODULE)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  shvan_msgs
)

include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

set(INCLUDE_DIR "include")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

# add_library(motor_controller SHARED src/motor_controller.cc)
# target_link_libraries(motor_controller ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} Eigen3::Eigen)

# add_library(quad_controller SHARED src/quadruped_controller.cc src/function_wrapper.cpp src/MassMatrix28_cpp.cpp)
# target_link_libraries(quad_controller ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} Eigen3::Eigen)

add_executable(IK_motor_command src/inverse_kinematics.cpp)
target_include_directories(IK_motor_command PRIVATE ${INCLUDE_DIR})
# target_link_libraries(trqCntrl ${GAZEBO_LIBRARIES} Eigen3::Eigen stdc++fs)# eng mat mx mex)
 #add_library(quad_controller SHARED src/quadruped_controller.cc src/function_wrapper.cpp)
 #target_link_libraries(quad_controller ${GAZEBO_LIBRARIES} Eigen3::Eigen)

#add_executable(trqCntrl src/torque_control.cc src/function_wrapper.cpp)
#target_include_directories(trqCntrl PRIVATE ${INCLUDE_DIR})
#target_link_libraries(trqCntrl ${GAZEBO_LIBRARIES} Eigen3::Eigen stdc++fs)# eng mat mx mex)
