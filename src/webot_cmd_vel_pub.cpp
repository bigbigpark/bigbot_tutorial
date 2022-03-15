/**
 * @brief Webot cmd_vel pub Node
 * @author SeongChang Park
 * @date 2022-03-15 16:49
 */
#include <ros/ros.h>
#include <cps_msgs/pose_id_array.h>
#include <cps_msgs/cmd_id.h>
#include <cps_msgs/cmd_id_array.h>

#include <iostream>
#include <string>
#include <vector>

using namespace std;

// This code will be deprecated soon
vector<string> robot_names = {"rbt1", "rbt2", "rbt3", "rbt4", "rbt5"};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "webot_cmd_vel_pub");
  ros::NodeHandle nh;
  ros::Rate r(10);

  ros::Publisher cmd_vel_pub = nh.advertise<cps_msgs::cmd_id_array>("/robot_cmd_vel_array", 1);
  auto cmd_vel_array_msg = cps_msgs::cmd_id_array();

  cmd_vel_array_msg.header.frame_id = "world";
  while(ros::ok())
  { 
    cmd_vel_array_msg.header.stamp = ros::Time::now();

    auto cmd_id = cps_msgs::cmd_id();

    for(auto& name : robot_names)
    {
      cmd_id.id = name;
      cmd_id.vel.linear.x = 0.5;
      cmd_id.vel.angular.z = 0.0;

      cmd_vel_array_msg.cmd_id_array.push_back(cmd_id);
    }
    cmd_vel_pub.publish(cmd_vel_array_msg);

    r.sleep();
  }

  return 0;
}