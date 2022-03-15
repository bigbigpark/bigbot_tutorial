cmake_minimum_required(VERSION 2.8.3)
project(bigbot_tutorial)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cps_msgs
)

find_package(cmake_modules REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  DEPENDS Eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(turningControl src/turningControl.cpp)
target_link_libraries(turningControl 
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)

add_executable(imuOdom src/imuOdom.cpp)
target_link_libraries(imuOdom 
  ${catkin_LIBRARIES}
  # ${Eigen_LIBRARIES}
)

add_executable(webot_cmd_vel_pub src/webot_cmd_vel_pub.cpp)
target_link_libraries(webot_cmd_vel_pub 
  ${catkin_LIBRARIES}
  # ${Eigen_LIBRARIES}
)
add_executable(webot_pose_sub src/webot_pose_sub.cpp)
target_link_libraries(webot_pose_sub 
  ${catkin_LIBRARIES}
  # ${Eigen_LIBRARIES}
)
