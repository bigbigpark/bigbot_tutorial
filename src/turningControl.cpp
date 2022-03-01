/*
* Authors: Seong-Chang Park
* Copyright (c) 2022, Seong-Chang, Park
*
* Reproduction in whole or in parts is prohibited without permission. 
*/
#include <ros/ros.h>
#include <iostream>
#include <tf/tf.h>
#include <vector>
#include <utility>
#include <vector>
#include <cmath>
// #include <queue>
// #include <string>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>

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
  Eigen::Vector3f prev_pose_;
  
  ros::Subscriber odom_sub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher path_pub_;
  nav_msgs::Odometry odom_msg_;
  geometry_msgs::Twist cmd_vel_msg_;
  
  void odomGazeboCallback(const nav_msgs::Odometry::ConstPtr& msg);

  vector< pair<double,double> > trg_wp;
  int cur_wp_idx;

  void autoPilot();
  double pi_to_pi(double angle);
  double dt;

  Eigen::Vector2f u_max;

  double R_acc;
  bool isEnd;

  nav_msgs::Path path_msg;
  geometry_msgs::PoseStamped poses;

  void stopRobot();
};

bool MyRobot::init()
{
  name_ = "bigbot";
  pose_ << 1, 0, 0;
  prev_pose_ << 0, 0, 0;

  odom_sub_ = nh_.subscribe("/bigbot/odom", 10, &MyRobot::odomGazeboCallback, this);
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/bigbot/cmd_vel", 10);
  path_pub_  = nh_.advertise<nav_msgs::Path>("/bigbot/path", 10);

  trg_wp.push_back( make_pair(10,  0) );
  trg_wp.push_back( make_pair(10, 10) );
  trg_wp.push_back( make_pair( 0, 10) );
  trg_wp.push_back( make_pair( 0,  0) );

  // trg_wp.push_back( make_pair( 0,-10) );
  // trg_wp.push_back( make_pair(-10,-10) );
  // trg_wp.push_back( make_pair(-10,  0) );
  // trg_wp.push_back( make_pair( 0,  0) );

  cur_wp_idx = 0;

  dt = 0.1;

  u_max << 1.0, 1.5;

  R_acc = 1.5;
  isEnd = false;

  return true;
}

void MyRobot::odomGazeboCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
  msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  pose_ << msg->pose.pose.position.x, msg->pose.pose.position.y, yaw;
  // cout << "[cur pose]: \n" << pose_ << endl;

  path_msg.header.frame_id = poses.header.frame_id = msg->header.frame_id;
  path_msg.header.stamp = poses.header.stamp = ros::Time::now();
  poses.pose = msg->pose.pose;
  path_msg.poses.push_back(poses);

  cout << "path size: " << path_msg.poses.size() << endl;

  path_pub_.publish(path_msg);
}

void MyRobot::autoPilot()
{
  pair<double,double> cur_wp = trg_wp[cur_wp_idx];
  double error_sq_x, error_sq_y, error_sq_theta;
  double error_dist;
  double error_los, error_dot_los, error_cte, error_dot_cte;
  Eigen::Vector2f u;

  // Calculate error
  error_sq_x = (pose_(0,0) - cur_wp.first) * (pose_(0,0) - cur_wp.first);
  error_sq_y = (pose_(1,0) - cur_wp.second) * (pose_(1,0) - cur_wp.second);
  error_dist = sqrt(error_sq_x + error_sq_y);

  // LOS error and its time derivative
  error_los = pi_to_pi(atan2(cur_wp.second - pose_(1,0), cur_wp.first - pose_(0,0)) - pose_(2,0));
  error_dot_los = error_los / dt;

  // CTE error and its time derivative
  error_cte = error_dist * sin( pi_to_pi(error_los - prev_pose_(2,0)) );
  error_dot_cte = error_cte / dt;

  // K control for linear velocity
  double Kp_x = 1.5;  // 1.5
  u(0,0) = Kp_x*error_dist;

  // KD control for angular velocity
  double Kp_yaw_los = 3, Kd_yaw_los = 0.001;    // 3,    0.001
  double Kp_yaw_cte = 0.01, Kd_yaw_cte = 0.001; // 0.01, 0.001
  u(1,0) = Kp_yaw_los*error_los + Kd_yaw_los*error_dot_los + Kp_yaw_cte*error_cte + Kd_yaw_cte*error_dot_cte;


  // Actuator saturation
  if (u(0,0) > u_max(0,0)) u(0,0) = u_max(0,0);
  else if(u(0,0) < -u_max(0,0)) u(0,0) = -u_max(0,0);
  if (u(1,0) > u_max(1,0)) u(1,0) = u_max(1,0);
  else if (u(1,0) < -u_max(1,0)) u(1,0) = -u_max(1,0);

  // Publish
  cmd_vel_msg_.linear.x  = u(0,0);
  cmd_vel_msg_.angular.z = u(1,0);
  cmd_vel_pub_.publish(cmd_vel_msg_);

  // Iterative
  prev_pose_ = pose_;

  // New wp or not?
  double dist = sqrt( ( cur_wp.first - pose_(0,0) ) * ( cur_wp.first - pose_(0,0) ) 
  + ( cur_wp.second - pose_(1,0) ) * ( cur_wp.second - pose_(1,0) ));
  if (dist < R_acc) cur_wp_idx++;

  if (cur_wp_idx == trg_wp.size()) isEnd = true;

  cout << "dist: " << dist << endl;
}

double MyRobot::pi_to_pi(double angle)
{
  while(angle >= M_PI)
		angle -= 2.*M_PI;
	while(angle < -M_PI)
		angle += 2.*M_PI;
	return angle;
}

void MyRobot::stopRobot()
{
  cmd_vel_msg_.linear.x = 0;
  cmd_vel_msg_.angular.z = 0;

  cmd_vel_pub_.publish(cmd_vel_msg_);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "turningControl");
  ros::NodeHandle nh;
  ros::Rate r(10);

  MyRobot mr;
  if (!mr.init()) return 0;

  while(ros::ok())
  {
    ros::spinOnce();

    mr.autoPilot();
    if (mr.isEnd) break;

    r.sleep();
  }

  ROS_WARN("END Criteria");
  mr.stopRobot();
  r.sleep();
  mr.stopRobot();

  return 0;
}