#include <iostream>
#include <geometry_msgs/Twist.h>
#include <tic_toc.h>
#include <ros/ros.h>

using namespace std;


int main(int argc, char** argv)
{
  ros::init(argc, argv, "makeConstantCmd");
  ros::NodeHandle nh;
  ros::Rate r(5);

  ros::Publisher pubCmdVel = nh.advertise<geometry_msgs::Twist>("/bigbot/cmd_vel", 1);
  geometry_msgs::Twist cmd_vel;

  TicToc t_make;
  
  double timestamp = 0.0;
  while(ros::ok())
  {
    ROS_INFO("timestamp: %f", timestamp);
    timestamp += 0.2;

    t_make.tic();
    if (timestamp < 10)
    {
      cmd_vel.linear.x  = 0.2;
      cmd_vel.angular.z = M_PI/2 * 0;
    }
    else if( timestamp < 20)
    {
      cmd_vel.linear.x  = 0.2;
      cmd_vel.angular.z = M_PI/2 * 1;
    }
    else if( timestamp < 30)
    {
      cmd_vel.linear.x  = 0.2;
      cmd_vel.angular.z = M_PI/2 * 2;
    }
    else
    {
      cmd_vel.linear.x  = 0.2;
      cmd_vel.angular.z = M_PI/2 * 3;
    }

    cout << cmd_vel.linear.x << " " <<  cmd_vel.angular.z << endl;


    pubCmdVel.publish(cmd_vel);
    cout << t_make.toc() <<  endl;

    r.sleep();
  }


  return 0;
}