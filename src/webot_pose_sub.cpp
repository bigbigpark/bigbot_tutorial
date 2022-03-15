/**
 * @brief Webot pose sub Node
 * @author SeongChang Park
 * @date 2022-03-15 16:55
 */
#include <ros/ros.h>
#include <cps_msgs/pose_id_array.h>

#include <iostream>
#include <string>
#include <vector>

using namespace std;

void webotCmdVelCallback(const cps_msgs::pose_id_array::ConstPtr& msg)
{
  int num_of_robots = msg->pose_id_array.size();

  cout << "size: " << num_of_robots << endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "webot_pose_sub");
  ros::NodeHandle nh;
  ros::Rate r(10);

  ros::Subscriber pose_sub = nh.subscribe("/robot_ground_truth_pose_array", 1, webotCmdVelCallback);

  ros::spin();

  return 0;
}