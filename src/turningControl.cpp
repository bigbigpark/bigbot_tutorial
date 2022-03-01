/*
* Authors: Seong-Chang Park
* Copyright (c) 2022, Seong-Chang, Park
*
* Reproduction in whole or in parts is prohibited without permission. 
*/
#include <ros/ros.h>
#include <iostream>
#include <tf/tf.h>
// #include <queue>
// #include <string>
// #include <vector>
#include <nav_msgs/Odometry.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>

using namespace std;

class MyRobot
{
public:
  MyRobot()
  {

  }
  ~MyRobot()
  {

  }

  bool init();

public:
  ros::NodeHandle nh_;

  std::string name_;
  Eigen::Vector3f pose_;
  
  ros::Subscriber odom_sub_;
  nav_msgs::Odometry odom_msg_;

  void odomGazeboCallback(const nav_msgs::Odometry::ConstPtr& msg);
};

bool MyRobot::init()
{
  name_ = "bigbot";
  pose_ << 1, 0, 0;

  odom_sub_ = nh_.subscribe("/bigbot/odom", 10, &MyRobot::odomGazeboCallback, this);

  return true;
}

void MyRobot::odomGazeboCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  // ROS_INFO("Callback()");

  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
  msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);


  pose_ << msg->pose.pose.position.x, msg->pose.pose.position.y, yaw;
  cout << "[cur pose]: \n" << pose_ << endl;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "turningControl");
  ros::NodeHandle nh;

  MyRobot mr;
  if (!mr.init()) return 0;


  ros::spin();


  return 0;
}