#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
@author SeongChang Park
@date 2022-03-15 20:40
@brief ROS Webot bridge
"""

from math import pi, cos, sin
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import numpy as np
import casadi as ca

# Weight values
Q_x = 100
Q_y = 100
Q_theta = 2000
R_v = 1
R_w = 1

# Define constant
STEP_HORIZON = 0.1    # time between steps in seconds
N = 10                # number of look ahead steps
ROBOT_DIAMETER = 0.3  # diameter of the robot
WHEEL_RADIUS = 1      # wheel radius
AXIS_DISTANCE = 0.3   # L in J Matrix (half robot x-axis length)
SIM_TIME = 30         # simulation time
V_MAX =  1.0
V_MIN = -1.0

class MPC_ROBOT():
  def __init__(self):
    # Robot states
    self.xpose = np.array([[0.0], [0.0], [0.0]])
    
    # Target robot states
    self.target_xpose = np.array([[10.0], [-10.0], [0.0]])
    
    # for ROS
    rospy.init_node('mpc_basic_node', anonymous=False)
    self.pose_sub = rospy.Subscriber('/bigbot/ground_truth_pose', Odometry, self.gazebo_pose_callback, queue_size=1)
    self.rate = rospy.Rate(10)
    
    # State
    
    # Controls
    
    # Objective function
    
    # Constraint
    
    # Optimization
    
    # Publish
    
    
    # ===========    Casadi implementation   ===================
    
    # 1. State symbolic variables
    self.x = ca.SX.sym('x')
    self.y = ca.SX.sym('y')
    self.theta = ca.SX.sym('theta')
    self.states = ca.vertcat(
      self.x, 
      self.y, 
      self.theta
    )
    self.n_states = self.states.numel()
    
    # 2. Control symbolic variables
    self.v = ca.SX.sym('v')
    self.w = ca.SX.sym('w')
    self.controls = ca.vertcat(
        self.v,
        self.w,
    )
    self.n_controls = self.controls.numel()
    
    # matrix containing all states over all time steps +1 (each column is a state vector)
    self.X = ca.SX.sym('X', self.n_states, N + 1)

    # matrix containing all control actions over all time steps (each column is an action vector)
    self.U = ca.SX.sym('U', self.n_controls, N)

    # coloumn vector for storing initial state and target state
    self.P = ca.SX.sym('P', self.n_states + self.n_states)

    # state weights matrix (Q_X, Q_Y, Q_THETA)
    self.Q = ca.diagcat(Q_x, Q_y, Q_theta)

    # controls weights matrix
    self.R = ca.diagcat(R_v, R_w)
    
    # euler discretization model (e.g. x2 = f(x1, v, t) = x1 + v * dt)
    self.rot_3d_z = ca.vertcat(
        ca.horzcat(cos(self.theta), 0),
        ca.horzcat(sin(self.theta), 0),
        ca.horzcat(         0,      1)
    )
    
    self.rhs = ca.mtimes(self.rot_3d_z, self.controls)
    
    # maps controls from [va, vb, vc, vd].T to [vx, vy, omega].T
    self.f = ca.Function('f', [self.states, self.controls], [self.rhs])
    
    self.cost_fn = 0  # cost function
    self.g = self.X[:, 0] - self.P[:self.n_states]  # constraints in the equation
    
    # runge kutta
    for k in range(N):
        st = self.X[:, k]
        con = self.U[:, k]
        self.cost_fn = self.cost_fn \
            + ca.mtimes( (st - self.P[self.n_states:]).T  , self.Q  , (st - self.P[self.n_states:]) ) \
            + ca.mtimes( con.T , self.R , con )
        # st_next = self.X[:, k+1]
        # k1 = self.f(st, con)
        # k2 = self.f(st + STEP_HORIZON/2*k1, con)
        # k3 = self.f(st + STEP_HORIZON/2*k2, con)
        # k4 = self.f(st + STEP_HORIZON * k3, con)
        # st_next_RK4 = st + (STEP_HORIZON / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
        # self.g = ca.vertcat(self.g, st_next - st_next_RK4)

  
  def gazebo_pose_callback(self, msg):
    self.xpose[0,0] = msg.pose.pose.position.x
    self.xpose[1,0] = msg.pose.pose.position.y
    self.xpose[2,0] = msg.pose.pose.position.z
    
if __name__ == '__main__':
  mpc_robot = MPC_ROBOT()
  
  while not rospy.is_shutdown():
    print('cur_pose: {}'.format(mpc_robot.xpose))
    # print(mpc_robot.rhs)
    pass