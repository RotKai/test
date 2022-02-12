#!/usr/bin/env python
# license removed for brevity

import rospy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import numpy as np
import copy
from quad_controllers_PM import pos_loop_MPC
from quad_properties_PM import *
from quad_dynamics_PM import quad_dyn_PM
from quad_trajectory_PM import circular_traj, corner_traj

# Select desired quadrotor type and create its object
m, A, B, uL, uU = quad_MATLAB_sim_PM()
quadrotor = quad_class_PM(m, A, B, uL, uU)
i = 0
g = 9.81
kp = 1
kv = 1
e3 = np.array([[0],[0],[1]])
freq = 100
N_MPC = 15
F = np.zeros(3)

odom_stored_msg = Odometry()

def set_point(N_MPC):
 	# k - time steps; freq - frequncy (Hz)
 	# Create storage
 	x = np.zeros((3,N_MPC))
 	v = np.zeros((3,N_MPC))
 	a = np.zeros((3,N_MPC))
 	# Setpoints
 	x_des = 0
 	y_des = 0
 	z_des = 1
 	# Position
 	x[0,:] =  x_des
 	x[1,:] =  y_des
 	x[2,:] =  z_des
 	# Velocity
 	v[0,:] = 0
 	v[1,:] = 0
 	v[2,:] = 0
 	# Acceleration
 	a[0,:] = 0
 	a[1,:] = 0
 	a[2,:] = 0   
 	return x


def circular_traj_1(time):
 	# k - time steps; freq - frequncy (Hz)
 	# Create storage
 	
 	x = np.zeros((3,int(np.size(time))))
 	v = np.zeros((3,int(np.size(time))))
 	a = np.zeros((3,int(np.size(time))))
 	# Set radius of circlular trajectory
 	r = 1 # (m)
 	# Set the cruise altitude
 	cruise_height = 0.5 # (m)
 	# Set velocity tangent to trajectory
 	lin_velocity = 0.8 # (m/s)
    
 	# Position
 	x[0,:] =  r * np.cos(time * lin_velocity / r)
 	x[1,:] =  r * np.sin(time * lin_velocity / r)
 	x[2,:] =  cruise_height
 	# Velocity
 	v[0,:] = - r * (lin_velocity / r) * np.sin(time * lin_velocity / r)
 	v[1,:] =   r * (lin_velocity / r) * np.cos(time* lin_velocity / r)
 	v[2,:] = 0
 	# Acceleration
 	a[0,:] = - r * (lin_velocity / r)**2 * np.cos(time * lin_velocity / r)
 	a[1,:] = - r * (lin_velocity / r)**2 * np.sin(time * lin_velocity / r)
 	a[2,:] = 0
    
 	return x, v, a


def odom_callback(msg):
	global odom_stored_msg
	odom_stored_msg = copy.deepcopy(msg)
    
def pd_control(time):
    global odom_stored_msg, F, m, g, e3
    # states
    xQ = np.array([[odom_stored_msg.pose.pose.position.x], 
                   [odom_stored_msg.pose.pose.position.y],
                   [odom_stored_msg.pose.pose.position.z]])
    vQ = np.array([[odom_stored_msg.twist.twist.linear.x],
                   [odom_stored_msg.twist.twist.linear.y],
                   [odom_stored_msg.twist.twist.linear.z]])
    print(xQ)
    # setpoint 
    # xd, vd, ad = set_point()
    # circular_traj
    xd, vd, ad = circular_traj_1(time)
    
    # control
    err_x = xQ-xd
    err_v = vQ-vd
    Fff = m*(g*e3 + ad)
    Fpd = -kp*err_x - kv*err_v
    F = Fff+Fpd
    print('force is :',F)
    
    
def mpc_control():
    
    global odom_stored_msg, F, i, quadrotor, freq, N_MPC
    # states
    x = np.array([ [odom_stored_msg.twist.twist.linear.x],
                   [odom_stored_msg.twist.twist.linear.y],
                   [odom_stored_msg.twist.twist.linear.z],
                   [odom_stored_msg.pose.pose.position.x], 
                   [odom_stored_msg.pose.pose.position.y],
                   [odom_stored_msg.pose.pose.position.z]])
    ref = set_point(N_MPC)
    # circular_traj
    feas, xMPC, uMPC = pos_loop_MPC(quadrotor,ref, N_MPC, i, freq, x.T[0])
    print('x is ', x.T[0])
    # control
    i += 1
    F = uMPC[:,0]
    print('force is :',F)   
    

def main():
    global i
    odom_sub = rospy.Subscriber("/white_falcon/odometry/mocap", Odometry, odom_callback)
    pub = rospy.Publisher('/white_falcon/thrust_force', TwistStamped, queue_size=10)
    rate = rospy.Rate(freq) # 100hz
    start_time = rospy.get_time()
    while i < 1500:

        print('i is ', i)
        mpc_control()
        print('F0 is ....', F[2])
        thrust = TwistStamped()
        thrust.twist.linear.x = F[0]
        thrust.twist.linear.y = F[1]
        thrust.twist.linear.z = float(F[2] + m*g)
        thrust.twist.angular.x = 0.0
        thrust.twist.angular.y = 0.0
        thrust.twist.angular.z = 0.0 
        rospy.loginfo(thrust)
        pub.publish(thrust)
        rate.sleep() 	


# def main():
    
#     odom_sub = rospy.Subscriber("/white_falcon/odometry/mocap", Odometry, odom_callback)
#     pub = rospy.Publisher('/white_falcon/thrust_force', TwistStamped, queue_size=10)
#     rate = rospy.Rate(freq) # 10hz
#     start_time = rospy.get_time()
#     while not rospy.is_shutdown():
    
#         current_time = rospy.get_time() - start_time
#         pd_control(current_time)
#         thrust = TwistStamped()
#         thrust.twist.linear.x = F[0][0]
#         thrust.twist.linear.y = F[1][0]
#         thrust.twist.linear.z = F[2][0]
#         thrust.twist.angular.x = 0.0
#         thrust.twist.angular.y = 0.0
#         thrust.twist.angular.z = 0.0
#         rospy.loginfo(thrust)
#         pub.publish(thrust)
#         rate.sleep() 	

if __name__ == '__main__':
    try:
        rospy.init_node('offboard_node')
        main()
    except rospy.ROSInterruptException:
        pass
     
