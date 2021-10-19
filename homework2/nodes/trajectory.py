#!/usr/bin/env python

## TRAJECTORY NODE ##
'''
The trajectory node uses the functions described in the calc_trajectory module to calculate the linear and angular velocity
of the turtle, and publishes the corresponding twist message to either turtlesim or turtlebot depending on the mode specified
in the launch file. Additionally, pause and resume services are defined to start and stop the turtle's motion while maintaining
the trajectory, and finally, a static transform is broadcasted between the world and odometry frames.

Custom Services:
    Name: /Pause Type: homework2/Pause ~ Stops turtle motion and records the pause time

    Name: /Resume Type: homework2/Resume ~ Restarts the turtle at the pause time 

Publishers:
    Name: turtle1/cmd_vel Type: geometry_msgs/Twist ~ Sends a twist message to move the turtlesim turtle

    Name: cmd_vel Type: geometry_msgs/Twist ~ Sends a twist message to move the TurtleBot robot

Broadcasters:
    Name: tf_static Type: tf2_msgs/TFMessage ~ Broadcasts the static transform between the world and odometry frames

Parameters: 
    Name: /Parameters ~ Figure 8 dimensions and period

    Name: /Trajectory/~freq ~ Private parameter for publishing frequency
'''

import rospy
import numpy as np
import tf2_ros
import tf
from homework2.calc_trajectory import trajectory
import geometry_msgs.msg
from geometry_msgs.msg import Twist,Vector3
from homework2.srv import pause, resume
from numpy import *
from sympy import *

def Pause_Turtle(null):
    '''
    Called with the Pause service. Publishes 0 velocities to stop turtle movement and sets paused to True. 
    Takes no arguments, returns the pause time.
    '''
    global paused
    global start_time
    global pause_time
    global mode

    pause_time = rospy.get_time() - start_time 
    
    if mode == 'sim':
        pub = rospy.Publisher('turtle1/cmd_vel',Twist,queue_size = 10)
    elif mode == 'real':
        pub = rospy.Publisher('/cmd_vel',Twist,queue_size = 10)
    else:
        rospy.loginfo('Uknown mode')
    
    pub.publish(Twist(Vector3(x=0,y=0,z=0),Vector3(x=0,y=0,z=0)))
    paused = True

    return pause_time

def Resume_Turtle(null):
    '''
    Called with the Resume service. Resets paused to false and calculates total pause time.
    Takes no arguments, returns the latest pause duration but adds to the value of total pause_duration.
    '''
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
    '''
    Broadcasts a static transform between the world and odomtery frames. No arguments, no returns.
    '''
    parameters = rospy.get_param("/Parameters")

    W = parameters[0]
    H = parameters[1]
    T = parameters[2]

    traj = trajectory(W,H,T)    
    
    theta0_world = traj.theta0()

    quat = tf.transformations.quaternion_from_euler(0,0,theta0_world)

    static_broadcaster = tf2_ros.StaticTransformBroadcaster()

    world_to_odom = geometry_msgs.msg.TransformStamped()

    world_to_odom.header.stamp = rospy.Time.now()
    world_to_odom.header.frame_id = "world"
    world_to_odom.child_frame_id = "odom"

    world_to_odom.transform.translation.x = 0
    world_to_odom.transform.translation.y = 0
    world_to_odom.transform.translation.z = 0
    world_to_odom.transform.rotation.x = quat[0]
    world_to_odom.transform.rotation.y = quat[1]
    world_to_odom.transform.rotation.z = quat[2]
    world_to_odom.transform.rotation.w = quat[3]

    static_broadcaster.sendTransform(world_to_odom)

def main():
    '''
    Imports the trajectory functions from the calc_trajectory modules and the W, H, and T values from the 
    trajectory parameters and uses the imports to publish a linear and angular velocity to the turtle. No
    arguments, no returns.
    '''
    global pause_time
    global restart
    global pause_duration
    global start_time
    global paused
    global mode

    f = rospy.get_param("~freq")
    r = rospy.Rate(f)

    parameters = rospy.get_param("/Parameters")

    if mode == 'sim':
        pub = rospy.Publisher('turtle1/cmd_vel',Twist,queue_size = 10)
    elif mode == 'real':
        pub = rospy.Publisher('cmd_vel',Twist,queue_size = 10)
    else:
        rospy.loginfo("Unknown mode")
    
    W = parameters[0]
    H = parameters[1]
    T = parameters[2]

    traj = trajectory(W,H,T)

    while not rospy.is_shutdown():
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
    global start_x
    global start_y
    global mode

    paused = True
    restart = 0
    pause_duration = 0
    pause_time = 0
    mode = rospy.get_param("mode")

    static_transform()

    start_time = rospy.get_time()
    main()

    rospy.spin()