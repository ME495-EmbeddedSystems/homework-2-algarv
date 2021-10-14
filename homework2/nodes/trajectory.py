#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist,Vector3
from turtlesim.msg import Pose


def callback(data):
    pub = rospy.Publisher('turtle1/cmd_vel',Twist,queue_size = 10)
    t = rospy.get_time()
    parameters = rospy.get_param("/Parameters")
    W = parameters[0]
    H = parameters[1]
    T = parameters[2]
    twist_value = Twist(Vector3(x=(W*np.pi/T)*np.cos(2*np.pi*t/T),y=(2*H*np.pi*t/T)*np.cos((4*np.pi*t)/T),z=0),Vector3(x=0,y=0,z=0))
    pub.publish(twist_value)

def main():
    parameters = rospy.get_param("/Parameters")
    rospy.Subscriber("turtle1/pose",Pose,callback)
    W = parameters[0]
    H = parameters[1]
    T = parameters[2]

if __name__ == '__main__':
    rospy.init_node('trajectory')
    rospy.Rate(500)
    main()
    rospy.spin()