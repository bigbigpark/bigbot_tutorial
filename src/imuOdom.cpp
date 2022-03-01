/*
* Authors: Seong-Chang Park
* Copyright (c) 2022, Seong-Chang, Park
*
* Reproduction in whole or in parts is prohibited without permission. 
*/

#include <iostream>
#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <tf/tf.h>

using namespace std;

class MyIMU
{
public:
  MyIMU(){};
  ~MyIMU(){};

  bool init();


public:
  ros::NodeHandle nh_;

  // Publihser & Subscriber
  ros::Subscriber imu_sub_;
  ros::Publisher imu_pub_;

  // Callback func.
  void imuGazeboCallback(const sensor_msgs::Imu::ConstPtr&msg);


  // ROS msgs
  sensor_msgs::Imu imu_msg_;


  // Other variables & functions
  double pi_to_pi(double angle);
};

bool MyIMU::init()
{
  imu_sub_ = nh_.subscribe("/bigbot/bigbot/imu", 10, &MyIMU::imuGazeboCallback, this);
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/bigbot/imu", 10);

}

void MyIMU::imuGazeboCallback(const sensor_msgs::Imu::ConstPtr&msg)
{
  cout << "---\n";
  cout << "linear_acc.x : " << msg->linear_acceleration.x << endl;
  cout << "linear_acc.y :" << msg->linear_acceleration.y << endl;
  cout << "linear_acc.z : " << msg->linear_acceleration.z << endl;
  cout << "angular_acc.x : " << msg->angular_velocity.x << endl;
  cout << "angular_acc.y : " << msg->angular_velocity.y << endl;
  cout << "angular_acc.z : " << msg->angular_velocity.z << endl;
  // cout << "angular_acc.z : " << msg->orientation. << endl;

  tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  cout << "roll  : " << pi_to_pi( roll * 180 / M_PI ) << endl;
  cout << "pitch : " << pi_to_pi( pitch * 180 / M_PI )<< endl;
  cout << "yaw   : " << pi_to_pi( yaw * 180 / M_PI )<< endl;

  // temp
  imu_msg_ = *msg;
  imu_msg_.header.frame_id = "/bigbot/base_link";
  imu_msg_.header.stamp = ros::Time::now();

  // temp
  imu_pub_.publish(imu_msg_);
}

double MyIMU::pi_to_pi(double angle)
{
  while (angle >= M_PI)
    angle -= 2*M_PI;
  while (angle < M_PI)
    angle += 2*M_PI;

	return angle;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imuOdom");
  ros::NodeHandle nh;

  MyIMU mi;
  mi.init();

  ros::spin();

  return 0;
}