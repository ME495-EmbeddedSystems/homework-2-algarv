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

def dynamic_transform(Pose):
    x_world = Pose.x
    y_world = Pose.y
    theta_world = Pose.theta
    v_world = Pose.linear_velocity
    w_world = Pose.angular_velocity  
    
    quat = tf.transformations.quaternion_from_euler(0,0,theta_world)

    broadcaster = tf2_ros.TransformBroadcaster()

    odom_to_baselink = geometry_msgs.msg.TransformStamped()

    odom_to_baselink.header.stamp = rospy.Time.now()
    odom_to_baselink.header.frame_id = "odom"
    odom_to_baselink.child_frame_id = "base_footprint"

    odom_to_baselink.transform.translation.x = x_world
    odom_to_baselink.transform.translation.y = y_world
    odom_to_baselink.transform.translation.z = 0
    odom_to_baselink.transform.rotation.x = quat[0]
    odom_to_baselink.transform.rotation.y = quat[1]
    odom_to_baselink.transform.rotation.z = quat[2]
    odom_to_baselink.transform.rotation.w = quat[3]

    broadcaster.sendTransform(odom_to_baselink)
      
    pub = rospy.Publisher('nav_msgs/Odometry',Odometry,queue_size = 10)
        
    quat = tf.transformations.quaternion_from_euler(0,0,theta_world)

    odom = Odometry()

    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = 'odom'
    odom.child_frame_id = 'base_footprint'
    odom.pose.pose.position = Point(x_world, y_world, 0)
    odom.pose.pose.orientation.x = quat[0]
    odom.pose.pose.orientation.y = quat[1]
    odom.pose.pose.orientation.z = quat[2]
    odom.pose.pose.orientation.w = quat[3] 
    odom.twist.twist = Twist(Vector3(x=v_world,y=0,z=0),Vector3(x=0,y=0,z=w_world))

    pub.publish(odom)

if __name__ == '__main__':
    rospy.init_node('simodom')
    rospy.Subscriber('/turtle1/pose',Pose,dynamic_transform)
    rospy.spin()