#!/usr/bin/python3

import rospy
import roslib
import tf

import PyKDL


import matplotlib.pyplot as plt
import numpy as np

import math
from math import sin, cos, pi
import numpy

# Messages
from std_msgs.msg import String, Int8, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist, Vector3Stamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from localizer_dwm1001.msg import Tag

#--------------------------------------

class DR_Publisher:
    def __init__(self):
        rospy.init_node('Dead_Reckoning_Publisher1', anonymous=True)
        #Pub
        self.xDR                    = rospy.Publisher("/debug/xDR",            Point, queue_size=50)    #50Hz
        self.xTrue                    = rospy.Publisher("/debug/xTrue",            Point, queue_size=50)    #50Hz     
        #Sub
        rospy.Subscriber('/vel_pub',      Twist,      self.vel_pub_callback)
        # rospy.Subscriber('/cmd_vel',      Twist,      self.cmd_vel_callback)
        rospy.Subscriber("/imu/rpy/filtered", Vector3Stamped, callback=self.imu_callback)
        # rospy.Subscriber('/cmd_vel',      Twist,      self.vel_pub_callback)
        rospy.Subscriber("/debug/pose_uwb", Point, callback=self.uwb_pose_callback)

        self.vel_pub = {'v':0.0 , 'w':0.0} #v robot (m/s)  && w robot (rad/s)
        # self.cmd_vel = {'v':0.0 , 'w':0.0}
        self.yaw_imu = 0.0

        #for debug
        self.last_time_DR  = rospy.Time.now()
        self.last_time_xTrue  = rospy.Time.now()

        self.pose = {'x': 0.0,'y': 0.0,'th': 0.0}
        self.posexDR = {'x': 0.0,'y': 0.0,'th': 0.0}
        self.posexTrue = {'x': 0.0,'y': 0.0,'th': 0.0}

        self.check_imu = False
        self.check_xDR = False
        self.firsttime = False

    def uwb_pose_callback(self,uwb_ekf_data: Point):
        if self.firsttime == False:
            self.pose = {'x': float(uwb_ekf_data.x),'y': float(uwb_ekf_data.y),'th': float(uwb_ekf_data.z)}
            self.posexDR = {'x': float(uwb_ekf_data.x),'y': float(uwb_ekf_data.y),'th': float(uwb_ekf_data.z)}
            self.posexTrue = {'x': float(uwb_ekf_data.x),'y': float(uwb_ekf_data.y),'th': float(uwb_ekf_data.z)}
            self.firsttime = True
    

    def update(self):
        pass
        # if self.check_xDR == True:      
        #     self.pub_xDR_debug(self.posexDR)
        #     self.check_xDR = False
        if self.check_imu == True:
            self.pub_xTrue_debug(self.posexTrue)
            self.check_imu = False
        self.pub_xDR_debug(self.posexDR)
        # self.pub_xTrue_debug(self.posexTrue)
    
    def pub_xDR_debug(self,pose):
        if self.firsttime == True:
            current_time = rospy.Time.now()
            dt = (current_time - self.last_time_DR).to_sec()   #deta_time
            self.last_time_DR = current_time                   #update time

            pose['x']   += self.vel_pub['v']*dt*cos(pose['th'] + 0.5*self.vel_pub['w']*dt)   # += v*TIME_SAMPE*cos(theta + 0.5*w*TIME_SAMPE);
            pose['y']   += self.vel_pub['v']*dt*sin(pose['th'] + 0.5*self.vel_pub['w']*dt)   # += v*TIME_SAMPE*sin(theta + 0.5*w*TIME_SAMPE);
            pose['th']  += self.vel_pub['w']*dt                                        # += w*TIME_SAMPE;
            
            pose['th'] = math.atan2(math.sin(pose['th']),math.cos(pose['th'])) # squash the orientation to between (-pi,pi)

            pose_ = Point()
            pose_.x = pose['x']
            pose_.y = pose['y']
            pose_.z = pose['th']
            self.xDR.publish(pose_)

    def pub_xTrue_debug(self,pose):
        if self.firsttime == True:
            current_time = rospy.Time.now()
            dt = (current_time - self.last_time_xTrue).to_sec()   #deta_time
            self.last_time_xTrue = current_time                   #update time

            pose['x']   += self.vel_pub['v']*dt*cos(-self.yaw_imu)   # += v*TIME_SAMPE*cos(theta + 0.5*w*TIME_SAMPE);
            pose['y']   += self.vel_pub['v']*dt*sin(-self.yaw_imu)   # += v*TIME_SAMPE*sin(theta + 0.5*w*TIME_SAMPE);
            pose['th']  = -self.yaw_imu                                       # += w*TIME_SAMPE;
            
            pose['th'] = math.atan2(math.sin(pose['th']),math.cos(pose['th'])) # squash the orientation to between (-pi,pi)

            pose_ = Point()
            pose_.x = pose['x']
            pose_.y = pose['y']
            pose_.z = pose['th']
            self.xTrue.publish(pose_)

    def vel_pub_callback(self,vel_pub_msg):
        pass
        self.vel_pub['v'] = vel_pub_msg.linear.x    # v robot (m/s)
        self.vel_pub['w'] = vel_pub_msg.angular.z   # w robot (rad/s)

    # def cmd_vel_callback(self,cmd_vel_msg):
    #     pass
    #     self.cmd_vel['v'] = cmd_vel_msg.linear.x    # v robot (m/s)
    #     self.cmd_vel['w'] = cmd_vel_msg.angular.z   # w robot (rad/s)

    def imu_callback(self,imu_msg):
        pass
        self.yaw_imu = imu_msg.vector.z
        self.check_imu = True
        
    
    def spin(self): #ok
        rospy.loginfo("[ROS] Start Node Dead_Reckoning_Publisher")
        rate = rospy.Rate(50)   
        while not rospy.is_shutdown():
            self.update()            
            rate.sleep()

# -------------Main--------------------
def main():
    dead_reckoning = DR_Publisher()
    dead_reckoning.spin()

if __name__ == '__main__':
    main()
