
#include <iostream>
#include <ros/ros.h>
#include <casadi/casadi.hpp>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>

using namespace std;
using namespace casadi;

class MPC_Robot
{
public:
  MPC_Robot()
  {

  }
  ~MPC_Robot()
  {

  }

  void init();

  // For constant
  const double T = 0.2;
  const double N = 10;
  const double ROBOT_DIAMETER = 0.3;
  const double V_MAX =  0.8; // linear velocity
  const double V_MIN = -0.8;
  const double W_MAX =  M_PI/4; // angular velocity
  const double W_MIN = -M_PI/4;

  // Weight matrix
  Eigen::Matrix3d Q;
  Eigen::Matrix2d R;

  // For Casadi
  // SX x;


private:
  Eigen::Vector3f xpose_;

};

void MPC_Robot::init()
{
  // Weight matrix
  Q << 1.0, 0.0, 0.0,
       0.0, 5.0, 0.0,
       0.0, 0.0, 0.1;
  R << 0.5, 0.0,
       0.0, 0.05;
}

int main(int argc, char** argv)
{
  // ros::init(argc, argv, "mpc_basic_node");
  // ros::NodeHandle nh;
  // ros::Rate r(10);

  // MPC_Robot mpc_robot;
  // mpc_robot.init();

  // cout << mpc_robot.Q << endl;
  // cout << mpc_robot.R << endl;

  casadi::SX x = casadi::SX::sym("x");
  cout << x << endl;


  return 0;
}