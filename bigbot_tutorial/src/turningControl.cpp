/**
 * @author SeongChang Park
 * @email scsc1125@gmail.com
 * @create date 2023-03-03 11:18:42
 * @modify date 2023-03-03 11:18:42
 */

#include <ros/ros.h>
#include <iostream>
#include <tf/tf.h>
#include <vector>
#include <utility>
#include <vector>
#include <cmath>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>

#define  NUM_OF_OBS  4

using namespace std;

class ArtificialPotentialField
{
public:
  ArtificialPotentialField(/* args */)
  {

  }
  ~ArtificialPotentialField()
  {

  }
private:
  friend class MyRobot;
  double K_att = 0.02;
  double K_rep = 3.0;
  double safety_region = 1.0;
  
  void calculate_att_force(Eigen::Vector2f& cur_pos, Eigen::Vector2f& goal_pos, Eigen::Vector2f& F_att)
  {
    double e_x = goal_pos(0,0) - cur_pos(0,0);
    double e_y = goal_pos(1,0) - cur_pos(1,0);
    auto dist = (goal_pos - cur_pos).norm();
    F_att << 0.0, 0.0;
    if (dist != 0.0)
    {
      F_att(0,0) += K_att * e_x/dist;
      F_att(1,0) += K_att * e_y/dist;
    }
  }

  void calculate_rep_force(Eigen::Vector2f& cur_pos, std::vector<Eigen::Vector2f>& obs_arr, Eigen::Vector2f& F_rep)
  {
    double e_x, e_y, dist;
    double new_ex, new_ey;
    F_rep << 0.0, 0.0;

    for(auto& obs : obs_arr)
    {
      e_x = obs(0,0) - cur_pos(0,0);
      e_y = obs(1,0) - cur_pos(1,0);
      dist = fabs( (obs - cur_pos).norm() - safety_region);
      
      new_ex = safety_region/(dist+safety_region) * e_x;
      new_ey = safety_region/(dist+safety_region) * e_y;

      double p = 0.1;
      std::cout << "dist: " << dist << std::endl;
      if (dist < p)
      {
        F_rep(0,0) -= K_rep * (1.0/dist - 1.0/p) * (1/(dist*dist)) * (new_ex/dist);
        F_rep(1,0) -= K_rep * (1.0/dist - 1.0/p) * (1/(dist*dist)) * (new_ey/dist);
      }
      else
      {
        F_rep(0,0) -= 0.0;
        F_rep(1,0) -= 0.0;
      }
    }
  }
};

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
  ros::Publisher generated_path_pub_;
  nav_msgs::Odometry odom_msg_;
  geometry_msgs::Twist cmd_vel_msg_;
  
  ros::Publisher vis_pub_;

  void odomGazeboCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void stopRobot();
  void autoPilot();
  double pi_to_pi(double angle);
  void makeObstacle();

  vector< pair<double,double> > trg_wp;

  double timestamp_ = 0.0;
  int cur_wp_idx;
  double dt;
  Eigen::Vector2f traj_xy;
  bool isTrajInit = false;
  Eigen::Vector2f u_max;

  double R_acc;
  bool isEnd;

  nav_msgs::Path path_msg;
  geometry_msgs::PoseStamped poses;

  nav_msgs::Path generated_path_msg;
  geometry_msgs::PoseStamped generated_poses;

  ArtificialPotentialField APF_;
  std::vector<Eigen::Vector2f> obs_arr_;
};

bool MyRobot::init()
{
  name_ = "bigbot";
  pose_ << 1, 0, 0;
  prev_pose_ << 0, 0, 0;

  odom_sub_ = nh_.subscribe("/rbt1/odom_ground_truth", 10, &MyRobot::odomGazeboCallback, this);
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/rbt1/jackal_velocity_controller/cmd_vel", 10);
  path_pub_  = nh_.advertise<nav_msgs::Path>("/bigbot/path", 10);
  vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>( "obstacle", 0);
  generated_path_pub_ = nh_.advertise<nav_msgs::Path>("/bigbot/gen_path", 10);

  trg_wp.push_back( make_pair(10, 0.0) );
  trg_wp.push_back( make_pair(10, 10) );
  trg_wp.push_back( make_pair( 0, 10) );
  trg_wp.push_back( make_pair( 0,  0) );

  // trg_wp.push_back( make_pair( 0,-10) );
  // trg_wp.push_back( make_pair(-10,-10) );
  // trg_wp.push_back( make_pair(-10,  0) );
  // trg_wp.push_back( make_pair( 0,  0) );

  obs_arr_.reserve(NUM_OF_OBS);
  obs_arr_.push_back(Eigen::Vector2f(5.0, -0.5));
  obs_arr_.push_back(Eigen::Vector2f(10.5, 5.0));
  obs_arr_.push_back(Eigen::Vector2f(5.0, 10.5));
  obs_arr_.push_back(Eigen::Vector2f(-0.5, 5.0));

  cur_wp_idx = 0;
  traj_xy << 0.0, 0.0;

  dt = 0.1;

  u_max << 1.5, M_PI/2; //  0.2, M_PI/2;

  R_acc = 1.0;
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

  if (!isTrajInit)
  {
    isTrajInit = true;
    traj_xy << pose_(0,0), pose_(1,0);
  }

  // path_msg.header.frame_id = poses.header.frame_id = msg->header.frame_id;
  path_msg.header.frame_id = poses.header.frame_id = "world";
  path_msg.header.stamp = poses.header.stamp = ros::Time::now();
  poses.pose = msg->pose.pose;
  path_msg.poses.push_back(poses);

  path_pub_.publish(path_msg);
}

void MyRobot::makeObstacle()
{
  visualization_msgs::MarkerArray marker_arr;

  int id = 0;
  for(auto& obs: obs_arr_)
  {
    visualization_msgs::Marker marker;

    marker.header.frame_id = "world"; // map frame 기준
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.id = id++;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.pose.position.x = obs(0,0); //노드의 x 좌표
    marker.pose.position.y = obs(1,0); //노드의 y 좌표
    // Points are green
    marker.color.g = 0.5;
    marker.color.a = 1.0;
    marker.scale.x = APF_.safety_region*2;
    marker.scale.y = APF_.safety_region*2;     
    marker_arr.markers.push_back(marker);
  }
  vis_pub_.publish(marker_arr);
}

void MyRobot::autoPilot()
{
  timestamp_ += 0.1;
  std::cout << "\n\n---\ntimestamp: " << timestamp_ << std::endl;
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

  // P control for linear velocity
  double Kp_x = 1.5;  // 1.5
  u(0,0) = Kp_x*error_dist;

  // PD control for angular velocity
  double Kp_yaw_los = 3, Kd_yaw_los = 0.001;    // 3,    0.001
  double Kp_yaw_cte = 0.01, Kd_yaw_cte = 0.001; // 0.01, 0.001
  u(1,0) = Kp_yaw_los*error_los + Kd_yaw_los*error_dot_los + Kp_yaw_cte*error_cte + Kd_yaw_cte*error_dot_cte;

  // Potential func
  Eigen::Vector2f cur_pose(pose_(0,0), pose_(1,0));
  Eigen::Vector2f goal_pose(cur_wp.first, cur_wp.second);
  Eigen::Vector2f F_att, F_rep;

  APF_.calculate_att_force(traj_xy, goal_pose, F_att);
  APF_.calculate_rep_force(traj_xy, obs_arr_, F_rep);

  std::cout << "F_att(0,0): " << F_att(0,0) << std::endl;
  std::cout << "F_att(1,0): " << F_att(1,0) << std::endl;
  std::cout << "F_rep(0,0): " << F_rep(0,0) << std::endl;
  std::cout << "F_rep(1,0): " << F_rep(1,0) << std::endl;

  traj_xy(0,0) += F_att(0,0) + F_rep(0,0);
  traj_xy(1,0) += F_att(1,0) + F_rep(1,0);

  std::cout << std::fixed << std::setprecision(2);
  std::cout << "traj_x: " << traj_xy(0,0) << std::endl;
  std::cout << "traj_y: " << traj_xy(1,0) << std::endl;

  // Publish generated path
  geometry_msgs::Pose pose_temp;
  pose_temp.position.x = traj_xy(0,0);
  pose_temp.position.y = traj_xy(1,0);
  generated_poses.pose = pose_temp;
  generated_path_msg.header.frame_id = "world";
  generated_path_msg.header.stamp = ros::Time::now();
  generated_path_msg.poses.push_back(generated_poses);
  generated_path_pub_.publish(generated_path_msg);

  // Actuator saturation
  if (u(0,0) > u_max(0,0)) u(0,0) = u_max(0,0);
  else if(u(0,0) < -u_max(0,0)) u(0,0) = -u_max(0,0);
  if (u(1,0) > u_max(1,0)) u(1,0) = u_max(1,0);
  else if (u(1,0) < -u_max(1,0)) u(1,0) = -u_max(1,0);

  // Publish
  cmd_vel_msg_.linear.x  = u(0,0);
  cmd_vel_msg_.angular.z = u(1,0);
  cmd_vel_pub_.publish(cmd_vel_msg_);

  // Publish marker
  makeObstacle();

  // Iterative
  prev_pose_ = pose_;

  // New wp or not?
  double dist = sqrt( ( cur_wp.first - pose_(0,0) ) * ( cur_wp.first - pose_(0,0) ) 
  + ( cur_wp.second - pose_(1,0) ) * ( cur_wp.second - pose_(1,0) ));

  auto d = (goal_pose - traj_xy).norm();

  if (dist < R_acc) cur_wp_idx++;
  // if (d < R_acc) cur_wp_idx++;
  std::cout << "remained dist: " << d << std::endl;

  if (cur_wp_idx == trg_wp.size()) isEnd = true;
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