/**
 * @file formationControl.cpp
 * @author Seongchang Park (scsc1125@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2022-06-08 17:14
 * @copyright Copyright (c) 2022
 * 
 */

#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <iomanip>
#include <algorithm>
#include <mutex>
#include <queue>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>


#define   END    "\033[0m"
#define   RED    "\033[1;31m"
#define   GREEN  "\033[1;32m"
#define   BLUE   "\033[1;34m"


using namespace std;

enum Shape
{
  VSHAPE,
  VERTICAL_LINE
};

struct Formation
{
  int mode = -1;
  Eigen::Vector3d offset; // x, y, theta
};

struct RobotInfo
{
  string name;
  Eigen::Vector3d xpose;
  Formation formation;
  Eigen::Vector3d ref_traj;
};

class FormationControl
{
public:
  FormationControl(ros::NodeHandle& nh) : nh_(nh)
  {
    rbt1_cmd_vel_pub_    = nh_.advertise<geometry_msgs::Twist>("/rbt1/jackal_velocity_controller/cmd_vel", 1);
    rbt2_cmd_vel_pub_    = nh_.advertise<geometry_msgs::Twist>("/rbt2/jackal_velocity_controller/cmd_vel", 1);
    rbt3_cmd_vel_pub_    = nh_.advertise<geometry_msgs::Twist>("/rbt3/jackal_velocity_controller/cmd_vel", 1);
    rbt1_pose_sub_       = nh_.subscribe("/rbt1/odom_ground_truth", 5, &FormationControl::rbt1PoseCallback, this);
    rbt2_pose_sub_       = nh_.subscribe("/rbt2/odom_ground_truth", 5, &FormationControl::rbt2PoseCallback, this);
    rbt3_pose_sub_       = nh_.subscribe("/rbt3/odom_ground_truth", 5, &FormationControl::rbt3PoseCallback, this);

    robot_.resize(num_of_robot_);
    robot_[0].name = "rbt1";
    robot_[1].name = "rbt2";
    robot_[2].name = "rbt3";

  }
  ~FormationControl()
  {

  }
  // function
  void process(int timestamp);

private:
  // Gain
  double Kp = 0.8;

  // ROS
  ros::NodeHandle nh_;
  ros::Publisher  rbt1_cmd_vel_pub_;
  ros::Publisher  rbt2_cmd_vel_pub_;
  ros::Publisher  rbt3_cmd_vel_pub_;
  ros::Subscriber rbt1_pose_sub_;
  ros::Subscriber rbt2_pose_sub_;
  ros::Subscriber rbt3_pose_sub_;

  void rbt1PoseCallback(const nav_msgs::Odometry::Ptr& msg);
  void rbt2PoseCallback(const nav_msgs::Odometry::Ptr& msg);
  void rbt3PoseCallback(const nav_msgs::Odometry::Ptr& msg);

  queue<nav_msgs::Odometry::Ptr> rbt1_odom_queue_;
  queue<nav_msgs::Odometry::Ptr> rbt2_odom_queue_;
  queue<nav_msgs::Odometry::Ptr> rbt3_odom_queue_;

  mutex mBuf;

  // Formation Control
  int num_of_robot_ = 3;
  vector<RobotInfo> robot_;
  Eigen::Vector3d swarm_centroid_;
  
  void setOneRobotPose(int index, const nav_msgs::Odometry::Ptr& odom);

  void generateFormationInfo(int mode);
  void generateSwarmCentroid(int timestamp);
  void generateRefTrajectory();

  vector<Eigen::Vector2f> computeCmdVel();

  void printAllRobotPosition();
  void printAllFormation();
  void printSwarmCentroid();
  void printRefTrajectory();
  void printAllCmdVel(vector<Eigen::Vector2f>& cmd_vel);

  void publishAllCmdVel(vector<Eigen::Vector2f>& cmd_vel);
};

void FormationControl::rbt1PoseCallback(const nav_msgs::Odometry::Ptr& msg)
{
  mBuf.lock();
  rbt1_odom_queue_.push(msg);
  mBuf.unlock();
}

void FormationControl::rbt2PoseCallback(const nav_msgs::Odometry::Ptr& msg)
{
  mBuf.lock();
  rbt2_odom_queue_.push(msg);
  mBuf.unlock();
}

void FormationControl::rbt3PoseCallback(const nav_msgs::Odometry::Ptr& msg)
{
  mBuf.lock();
  rbt3_odom_queue_.push(msg);
  mBuf.unlock();
}

void FormationControl::publishAllCmdVel(vector<Eigen::Vector2f>& cmd_vel)
{
  vector<geometry_msgs::Twist> twist(num_of_robot_);

  for(auto& cmd : cmd_vel)
  {
    for(auto& tw : twist)
    {
      tw.linear.x  = cmd(0);
      tw.angular.z = cmd(1);
    }
  }
  rbt1_cmd_vel_pub_.publish(twist[0]);
  rbt2_cmd_vel_pub_.publish(twist[1]);
  rbt3_cmd_vel_pub_.publish(twist[2]);
}

vector<Eigen::Vector2f> FormationControl::computeCmdVel()
{
  vector<Eigen::Vector2f> cmd_vel(num_of_robot_);

  Eigen::Vector2f error_xy;  // x_err, y_err, th_err
  double error_th = 0.0;
  int index = 0;
  for(auto& robot: robot_)
  {
    error_xy(0) = robot.ref_traj(0) - robot.xpose(0);
    error_xy(1) = robot.ref_traj(1) - robot.xpose(1);
    error_th    = robot.ref_traj(2) - robot.xpose(2);
    double error_dist = error_xy.norm();

    cmd_vel[index](0) = Kp*error_dist;

    index++;
  }

  return cmd_vel;
}

void FormationControl::generateRefTrajectory()
{
  for(auto& robot : robot_)
  {
    robot.ref_traj(0) = swarm_centroid_(0) + robot.formation.offset(0);
    robot.ref_traj(1) = swarm_centroid_(1) + robot.formation.offset(1);
    robot.ref_traj(2) = swarm_centroid_(2) + robot.formation.offset(2);
  }
}

void FormationControl::generateSwarmCentroid(int timestamp)
{
  // Set swarm_centroid_
  swarm_centroid_(0) += timestamp*0.01;
  swarm_centroid_(1) += 0.0;
  swarm_centroid_(2) += 0.0;
}

void FormationControl::generateFormationInfo(int mode)
{
  switch (mode)
  {
  case VSHAPE:
    //    2
    //    * 1
    //    3
    robot_[0].formation.offset(0) =   2.0;
    robot_[0].formation.offset(1) =   0.0;
    robot_[1].formation.offset(0) =   0.0;
    robot_[1].formation.offset(1) =   2.0;
    robot_[2].formation.offset(0) =   0.0;
    robot_[2].formation.offset(1) =  -2.0;
    break;

  case VERTICAL_LINE:
    //   2
    //   1(*)
    //   3
    robot_[0].formation.offset(0) =   0.0;
    robot_[0].formation.offset(1) =   0.0;
    robot_[1].formation.offset(0) =   0.0;
    robot_[1].formation.offset(1) =   2.0;
    robot_[2].formation.offset(0) =   0.0;
    robot_[2].formation.offset(1) =  -2.0;
    break;
  
  default:
    break;
  }

}

void FormationControl::setOneRobotPose(int index, const nav_msgs::Odometry::Ptr& odom)
{
  robot_[index].xpose(0) = odom->pose.pose.position.x;
  robot_[index].xpose(1) = odom->pose.pose.position.y;

  tf::Quaternion q(
    odom->pose.pose.orientation.x,
    odom->pose.pose.orientation.y,
    odom->pose.pose.orientation.z,
    odom->pose.pose.orientation.w
  );
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  robot_[index].xpose(2) = yaw*180.0/M_PI;
}

void FormationControl::printAllRobotPosition()
{
  cout << fixed << setprecision(4);
  cout << "xpose 1: \n" << robot_[0].xpose << endl;
  cout << "xpose 2: \n" << robot_[1].xpose << endl;
  cout << "xpose 3: \n" << robot_[2].xpose << endl;
}

void FormationControl::printAllFormation()
{
  cout << fixed << setprecision(4);
  cout << "formation 1:\n" << robot_[0].formation.offset << endl;
  cout << "formation 2:\n" << robot_[1].formation.offset << endl;
  cout << "formation 3:\n" << robot_[2].formation.offset << endl;
}

void FormationControl::printSwarmCentroid()
{
  cout << fixed << setprecision(4);
  cout << "swarm centroid:\n"<< swarm_centroid_ <<endl;
}

void FormationControl::printRefTrajectory()
{
  cout << fixed << setprecision(4);
  cout << "ref_traj 1:\n" << robot_[0].ref_traj << endl;
  cout << "ref_traj 2:\n" << robot_[1].ref_traj << endl;
  cout << "ref_traj 3:\n" << robot_[2].ref_traj << endl;
}

void FormationControl::printAllCmdVel(vector<Eigen::Vector2f>& cmd_vel)
{
  cout << fixed << setprecision(4);
  cout << "cmd 1:\n" << cmd_vel[0] << endl;
  cout << "cmd 2:\n" << cmd_vel[1] << endl;
  cout << "cmd 3:\n" << cmd_vel[2] << endl;
}

void FormationControl::process(int timestamp)
{
  if (!rbt1_odom_queue_.empty() && !rbt2_odom_queue_.empty() && !rbt3_odom_queue_.empty())
  {
    cout << "\n\n" << GREEN << "  process  " << END << endl;
    mBuf.lock();
    auto rbt1_odom = rbt1_odom_queue_.front();
    rbt1_odom_queue_.pop();

    auto rbt2_odom = rbt2_odom_queue_.front();
    rbt2_odom_queue_.pop();

    auto rbt3_odom = rbt3_odom_queue_.front();
    rbt3_odom_queue_.pop();
    mBuf.unlock();

    setOneRobotPose(0, rbt1_odom);
    setOneRobotPose(1, rbt2_odom);
    setOneRobotPose(2, rbt3_odom);

    generateSwarmCentroid(timestamp);
    generateFormationInfo(VSHAPE);
    generateRefTrajectory();

    auto cmd_vel = computeCmdVel();

    printAllRobotPosition();
    // printAllFormation();
    // printSwarmCentroid();
    printRefTrajectory();
    printAllCmdVel(cmd_vel);

    publishAllCmdVel(cmd_vel);
  }
  else
  {
    ROS_WARN("rbt N's odom queue is empty! break the loop...");
  }

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "formationControl");
  ros::NodeHandle nh("~");
  ros::Rate r(10);

  FormationControl FC(nh);

  double timestamp = 0.0;
  while(ros::ok())
  {
    timestamp += 0.1;
    ros::spinOnce();

    FC.process(timestamp);

    r.sleep();
  }

  return 0;
}