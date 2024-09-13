#!/usr/bin/env python3

import rospy, tf
import roslib
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, Quaternion
from geometry_msgs.msg import TransformStamped, Vector3, PoseWithCovarianceStamped, Vector3Stamped
import PyKDL
from numpy import sin, cos, deg2rad
from math import atan2

vel  = {'v': 0.0 , 'w': 0.0}
pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
allow_initialpose_pub = False
ekf_uwb_done = False
uwb_x = 0
uwb_y = 0
uwb_yaw = 0

def publish_odometry(position, rotation):
    odom = Odometry()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = 'odom'
    odom.child_frame_id = 'base_footprint'
    odom.pose.pose.position = Point(*position)
    odom.pose.pose.orientation = Quaternion(*rotation)
    odom.twist.twist.linear  = Vector3(vel['v'], 0, 0)
    odom.twist.twist.angular = Vector3(0, 0, vel['w'])

    odom_pub.publish(odom)

def transform_odometry(position, rotation):  
    trans = TransformStamped()
    trans.header.stamp = rospy.Time.now()
    trans.header.frame_id = 'odom'
    trans.child_frame_id = 'base_footprint'
    trans.transform.translation = Vector3(*position)
    trans.transform.rotation = Quaternion(*rotation)

    br.sendTransformMessage(trans)

def publisher_initialpose(position, rotation):
    initialpose = PoseWithCovarianceStamped()
    initialpose.header.stamp = rospy.Time.now()
    initialpose.header.frame_id = 'map'
    initialpose.pose.pose.position = Point(*position)
    initialpose.pose.pose.orientation = Quaternion(*rotation)

    init_pose_pub.publish(initialpose)

def subscriber_uwb_imu_ekf_callback(uwb_ekf_data: Point):
    global ekf_uwb_done, uwb_x, uwb_y, uwb_yaw
    ekf_uwb_done = True
    uwb_x = uwb_ekf_data.x
    uwb_y = uwb_ekf_data.y
    uwb_yaw = uwb_ekf_data.z

def dead_reckoning(pose):
    pose['x'] = uwb_x
    pose['y'] = uwb_y
    pose['yaw'] = uwb_yaw
    return pose

def main():
    global odom_pub, init_pose_pub, br, previous_time, pose
    rospy.init_node('node_odom_ekf_uwb_imu')
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
    rospy.Subscriber('/debug/pose_uwb_imu', Point, subscriber_uwb_imu_ekf_callback)
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(50)
    previous_time = rospy.Time.now()
    while not rospy.is_shutdown():
        if  ekf_uwb_done:
            pose = dead_reckoning(pose)
            rospy.loginfo(pose)
            position = (pose['x'], pose['y'], 0)
            rotation = PyKDL.Rotation.RPY(0, 0, pose['yaw']).GetQuaternion()
            publish_odometry(position, rotation)
            transform_odometry(position, rotation)
        rate.sleep()

if __name__ == '__main__':
    main()


