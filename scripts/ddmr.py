#!/usr/bin/env python3

import rospy, tf
import roslib
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Point
from geometry_msgs.msg import Vector3,  Vector3Stamped
import PyKDL
import numpy as np
from numpy import sin, cos, deg2rad
from math import atan2

vel  = {'v': float(0) , 'w': float(0)}
pose = {'x': float(0), 'y': float(0), 'yaw': float(0)}
imu_done = False
vel_done = False
imu_yaw = 0


def publish_odometry(position):
    odom = Vector3()
    odom = Point(*position)
    odom_pub.publish(odom)

def subscriber_vel_callback(vel_data):
    global vel_done, vel
    vel_done = True
    vel['v'] = vel_data.linear.x
    vel['w'] = vel_data.angular.z

# def subscriber_imu_callback(imu_data):
#     global imu_done, imu_yaw
#     imu_done = True
#     imu_yaw = imu_data.vector.z

def dead_reckoning(pose):
    global previous_time
    current_time = rospy.Time.now()
    dt = (current_time - previous_time).to_sec()
    previous_time = current_time
    pose['x']   += vel['v']*dt*cos(pose['yaw'] + 0.5*vel['w']*dt)
    pose['y']   += vel['v']*dt*sin(pose['yaw'] + 0.5*vel['w']*dt)
    pose['yaw']  += vel['w']*dt
    # pose['yaw'] = atan2(sin(pose['yaw']), cos(pose['yaw']))
    # pose['x'] += vel['v']*dt*cos(-imu_yaw)
    # pose['y'] += vel['v']*dt*sin(-imu_yaw)
    # pose['yaw'] = -imu_yaw   
    return pose

def main():
    global odom_pub, previous_time, pose
    rospy.init_node('node_odom')
    odom_pub = rospy.Publisher('/odom', Vector3, queue_size=10)
    rospy.Subscriber('/cmd_vel', Twist, subscriber_vel_callback)
    # rospy.Subscriber('/imu/rpy/filtered', Vector3Stamped, subscriber_imu_callback)
    rate = rospy.Rate(50)
    previous_time = rospy.Time.now()
    while not rospy.is_shutdown():
        # if not imu_done:
        #     start_imu_pub.publish(Bool(True))
        # if vel_done and imu_done:
        if vel_done:
            pose = dead_reckoning(pose)
            rospy.loginfo(pose)
            position = (float(pose['x']), float(pose['y']), float(pose['yaw']))
            publish_odometry(position)
        rate.sleep()

if __name__ == '__main__':
    main()


#!/usr/bin/env python3

import rospy, tf
import roslib
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Point
from geometry_msgs.msg import Vector3,  Vector3Stamped
import PyKDL
from numpy import sin, cos, deg2rad
from math import atan2

vel  = {'v': float(0) , 'w': float(0)}
pose = {'x': float(0), 'y': float(0), 'yaw': float(0)}
imu_done = False
vel_done = False
imu_yaw = 0


def publish_odometry(position):
    odom = Vector3()
    odom = Point(*position)
    odom_pub.publish(odom)

def subscriber_vel_callback(vel_data):
    global vel_done, vel
    vel_done = True
    vel['v'] = vel_data.linear.x
    vel['w'] = vel_data.angular.z

# def subscriber_imu_callback(imu_data):
#     global imu_done, imu_yaw
#     imu_done = True
#     imu_yaw = imu_data.vector.z

def dead_reckoning(pose):
    global previous_time
    current_time = rospy.Time.now()
    dt = (current_time - previous_time).to_sec()
    previous_time = current_time
    pose['x']   += vel['v']*dt*cos(pose['yaw'] + 0.5*vel['w']*dt)
    pose['y']   += vel['v']*dt*sin(pose['yaw'] + 0.5*vel['w']*dt)
    pose['yaw']  += vel['w']*dt
    # pose['yaw'] = atan2(sin(pose['yaw']), cos(pose['yaw']))
    # pose['x'] += vel['v']*dt*cos(-imu_yaw)
    # pose['y'] += vel['v']*dt*sin(-imu_yaw)
    # pose['yaw'] = -imu_yaw   
    return pose

def main():
    global odom_pub, previous_time, pose
    rospy.init_node('node_odom')
    odom_pub = rospy.Publisher('/odom', Vector3, queue_size=10)
    rospy.Subscriber('/vel_pub', Twist, subscriber_vel_callback)
    # rospy.Subscriber('/imu/rpy/filtered', Vector3Stamped, subscriber_imu_callback)
    rate = rospy.Rate(200)
    previous_time = rospy.Time.now()
    while not rospy.is_shutdown():
        # if not imu_done:
        #     start_imu_pub.publish(Bool(True))
        # if vel_done and imu_done:
        if vel_done:
            pose = dead_reckoning(pose)
            rospy.loginfo(pose)
            position = (float(pose['x']), float(pose['y']), float(pose['yaw']))
            publish_odometry(position)
        rate.sleep()

if __name__ == '__main__':
    main()



