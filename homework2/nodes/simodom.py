#!/usr/bin/env python
import rospy
import tf2_ros
import tf
import math
from tf_conversions import transformations
from homework2.calc_trajectory import trajectory
import geometry_msgs.msg
from geometry_msgs.msg import Twist,Vector3, Point
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry


def get_world(Pose):
    broadcaster = tf2_ros.TransformBroadcaster()
    while not rospy.is_shutdown():
        x_world = Pose.x
        y_world = Pose.y
        theta_world = Pose.theta

        v_world = Pose.linear_velocity
        w_world = Pose.angular_velocity        
        
        pub = rospy.Publisher('nav_msgs/Odometry',Odometry,queue_size = 10)
        quat = tf.transformations.quaternion_from_euler(theta_world,0,0)

        odom = Odometry()

        odom.header.stamp = rospy.get_time()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position = Point(x_world, y_world, 0)
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3] 
        odom.twist = Twist(Vector3(x=v_world,y=0,z=0),Vector3(x=0,y=0,z=w_world))

        pub.publish(odom)

        broadcaster.sendTransform('base_link')

if __name__ == '__main__':
    rospy.init_node('simodom')
    rospy.Subscriber('/turtle1/pose',Pose,get_world)
    rospy.spin()