#!/usr/bin/env python
import rospy
import numpy as np
import homework2.calc_trajectory
from homework2.calc_trajectory import trajectory
from geometry_msgs.msg import Twist,Vector3
from numpy import *
from sympy import *
  

if __name__ == '__main__':
    rospy.init_node('trajectory')
    parameters = rospy.get_param("/Parameters")
    W = parameters[0]
    H = parameters[1]
    T = parameters[2]
    r = rospy.Rate(200)
    traj = trajectory(W,H,T)
    while not rospy.is_shutdown():
        pub = rospy.Publisher('turtle1/cmd_vel',Twist,queue_size = 10)
        t = rospy.get_time()
        #x_dot = (W*np.pi/T)*np.cos(2*np.pi*t/T)
        #y_dot = (2*H*np.pi/T)*np.cos((4*np.pi*t)/T)
        #x=sqrt(x_dot**2 + y_dot**2)
        #w = -(4*pi*H*W*(sin((6*pi*t)/T)+3*sin((2*pi*t)/T)))/(T*(4*H**2*(cos((8*pi*t)/T)+1)+W**2*(cos((4*pi*t)/T)+1)))
        v = traj.linear_velocity(t)
        w = traj.angular_velocity(t)
        twist_value = Twist(Vector3(x=v,y=0,z=0),Vector3(x=0,y=0,z=w))
        pub.publish(twist_value)
        r.sleep()  
    rospy.spin()