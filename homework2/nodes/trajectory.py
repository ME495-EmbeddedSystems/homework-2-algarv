#!/usr/bin/env python
import rospy
import numpy as np
import tf2_ros
from homework2.calc_trajectory import trajectory
import geometry_msgs.msg
from geometry_msgs.msg import Twist,Vector3
from homework2.srv import pause, resume
from numpy import *
from sympy import *

def Pause_Turtle(null):
    global paused
    global start_time
    global pause_time

    pause_time = rospy.get_time() - start_time 
    pub = rospy.Publisher('turtle1/cmd_vel',Twist,queue_size = 10)
    pub.publish(Twist(Vector3(x=0,y=0,z=0),Vector3(x=0,y=0,z=0)))
    paused = True

    return pause_time

def Resume_Turtle(null):
    global pause_duration
    global start_time
    global paused
    global restart 

    old_pause_duration = pause_duration 

    if paused == True:
        pause_duration = (rospy.get_time() - start_time - pause_time) + pause_duration
        paused = False
        restart = 1
    else: 
        pause_duration = pause_duration
        
    return pause_duration - old_pause_duration

def static_transform():
    parameters = rospy.get_param("/Parameters")

    W = parameters[0]
    H = parameters[1]
    T = parameters[2]

    traj = trajectory(W,H,T)    
    
    theta0_world = traj.theta0

    static_broadcaster = tf2_ros.TransformBroadcaster()

    world_to_odom = geometry_msgs.msg.TransformStamped()

    world_to_odom.header.stamp = rospy.Time.now()
    world_to_odom.header.frame_id = "world"
    world_to_odom.child_frame_id = "base_link"

    world_to_odom.transform.translation.x = 0
    world_to_odom.transform.translation.y = 0
    world_to_odom.transform.translation.z = 0
    world_to_odom.transform.rotation.x = 0
    world_to_odom.transform.rotation.y = 0
    world_to_odom.transform.rotation.z = theta0_world
    world_to_odom.transform.rotation.w = 0

    static_broadcaster.sendTransform(world_to_odom)

def main():
    global pause_time
    global restart
    global pause_duration
    global start_time
    global paused

    r = rospy.Rate(50)

    parameters = rospy.get_param("/Parameters")
    pub = rospy.Publisher('turtle1/cmd_vel',Twist,queue_size = 10)

    W = parameters[0]
    H = parameters[1]
    T = parameters[2]

    traj = trajectory(W,H,T)

    while not rospy.is_shutdown():
        print('Hello')
        if not paused:
            if restart == 1:
                t = pause_time
                restart = 0
            else:
                t = rospy.get_time() - start_time - pause_duration


            v = traj.linear_velocity(t)
            w = traj.angular_velocity(t)
            twist_value = Twist(Vector3(x=v,y=0,z=0),Vector3(x=0,y=0,z=w))

            pub.publish(twist_value)

            r.sleep()

if __name__ == '__main__':
    rospy.init_node('trajectory')

    rospy.Service('Pause',pause,Pause_Turtle)
    rospy.Service('Resume',resume,Resume_Turtle)

    global paused
    global restart
    global start_time
    global pause_time
    global pause_duration

    paused = False
    restart = 0
    pause_duration = 0
    pause_time = 0
    
    static_transform()

    start_time = rospy.get_time()
    main()

    rospy.spin()