#!/usr/bin/python3

import rospy
import roslib
import tf

import PyKDL

import numpy as np
import math
from math import sin, cos, pi

from std_msgs.msg import String, Int8, Float32
from localizer_dwm1001.msg import Tag
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, Vector3Stamped


class UWB_Position:
    def __init__(self):
        rospy.init_node('Node_UWB_position', anonymous=True)
        self.dwm1001_position_pub = rospy.Publisher("/dwm1001/position",            Point, queue_size=50)
        self.dwm1001_position_ekf_pub = rospy.Publisher("/dwm1001/position_ekf",            Point, queue_size=50)

        rospy.Subscriber('/agv/uwb/tag1', Tag, self.uwb_l_callback)
        rospy.Subscriber('/agv/uwb/tag2', Tag, self.uwb_r_callback)
        rospy.Subscriber('/vel_pub', Twist, self.vel_callback)

        self.time_ekf_update = rospy.Time.now()

        self.dwm1001_pos = np.zeros((3, 1))
        self.dwm1001_pos_ekf = np.zeros((3, 1))

        self.PEst_uwb_r = np.eye(3)*10 
        self.xEst_uwb_r = np.zeros((3, 1))

        self.PEst_uwb_l = np.eye(3)*10 
        self.xEst_uwb_l = np.zeros((3, 1))

        self.vel ={'v':0.0,'w':0.0}
        self.vel_uwb_r = {'v_r': 0.0,'w_r': 0.0}
        self.vel_uwb_l = {'v_l': 0.0,'w_l': 0.0}

        # self.uwb_distance = 0.4
        self.uwb_distance = 0.5

        self.pose_uwb_r = {'x': 0.0,'y': 0.0,'th': 0.0}############ can do thuc te
        self.pose_uwb_l = {'x': 0.0,'y': 0.0,'th': 0.0}############

        self.xEst_uwb_r[0,0] = self.pose_uwb_r['x'] 
        self.xEst_uwb_r[1,0] = self.pose_uwb_r['y']
        self.xEst_uwb_r[2,0] = self.pose_uwb_r['th']

        self.xEst_uwb_l[0,0] = self.pose_uwb_l['x']
        self.xEst_uwb_l[1,0] = self.pose_uwb_l['y']
        self.xEst_uwb_l[2,0] = self.pose_uwb_l['th']

        self.uwb_l = np.zeros((3, 1))
        self.uwb_r = np.zeros((3, 1))
        self.check_uwb_l = False
        self.check_uwb_r = False

    def update(self):
        if self.check_uwb_l == True and self.check_uwb_r == True:
            pass
            self.pub_dwm1001_pos(self.dwm1001_pos)
            self.pub_dwm1001_pos_ekf(self.dwm1001_pos_ekf)
            self.check_uwb_l = False
            self.check_uwb_r = False
    
    def pub_dwm1001_pos(self, dwm1001_pos):
        pos_dwm1001_msg = Point()
        pos_dwm1001_msg.x = (self.uwb_l[0,0] + self.uwb_r[0,0])/2
        pos_dwm1001_msg.y = (self.uwb_l[1,0] + self.uwb_r[1,0])/2
        pos_dwm1001_msg.z = math.atan2(-(self.uwb_l[0,0]-self.uwb_r[0,0]),self.uwb_l[1,0]-self.uwb_r[1,0])
        self.dwm1001_position_pub.publish(pos_dwm1001_msg)
        rospy.loginfo("x = " + str(pos_dwm1001_msg.x) + ", " + "y = " + str(pos_dwm1001_msg.y)+ ", " + "z = " + str(pos_dwm1001_msg.z))

    def pub_dwm1001_pos_ekf(self, dwm1001_pos_ekf):
        pos_dwm1001_ekf_msg = Point()
        pos_dwm1001_ekf_msg.x = (self.xEst_uwb_l[0,0] + self.xEst_uwb_r[0,0])/2
        pos_dwm1001_ekf_msg.y = (self.xEst_uwb_l[1,0] + self.xEst_uwb_r[1,0])/2
        pos_dwm1001_ekf_msg.z = math.atan2(-(self.xEst_uwb_l[0,0]-self.xEst_uwb_r[0,0]),self.xEst_uwb_l[1,0]-self.xEst_uwb_r[1,0]) #atan2
        self.dwm1001_position_ekf_pub.publish(pos_dwm1001_ekf_msg)
        # rospy.loginfo("x = " + str(pos_dwm1001_ekf_msg.x) + ", " + "y = " + str(pos_dwm1001_ekf_msg.y)+ ", " + "z = " + str(pos_dwm1001_ekf_msg.z))
    def vel_callback(self,vel_msg):
        pass
        self.vel['v'] = vel_msg.linear.x    # v robot (m/s)
        self.vel['w'] = vel_msg.angular.z   # w robot (rad/s)

    def uwb_l_callback(self, l_uwb_msg):
        pass
        current_time = rospy.Time.now()
        Ts = (current_time - self.time_ekf_update).to_sec()
        self.time_ekf_update = current_time

        w_l = self.vel['w']
        v_l = self.vel['v'] - w_l*self.uwb_distance/2
        self.vel_uwb_l = {'v_l': v_l,'w_l': w_l}
        u_l = np.array([[self.vel_uwb_l['v_l']], [self.vel_uwb_l['w_l']]])
        # rospy.loginfo("v_l = " + str(u_l[0,0]) + ", " + "w_l = " + str(u_l[1,0]))
        self.Q = np.diag([0.01, 0.01, 0.02]) ** 2
        self.R_UWB      = np.diag([0.31, 0.31]) ** 2
        # if self.check_uwb_l == True:
        z = np.array([[l_uwb_msg.x],[l_uwb_msg.y]])
        self.xEst_uwb_l, self.PEst_uwb_l = self.ekf_estimation_uwb(self.xEst_uwb_l, self.PEst_uwb_l, z, u_l, Ts)
        self.pose_uwb_l['x'] = self.xEst_uwb_l[0,0]
        self.pose_uwb_l['y'] = self.xEst_uwb_l[1,0]
        self.pose_uwb_l['th'] = self.xEst_uwb_l[2,0]
        self.uwb_l[0,0] = l_uwb_msg.x
        self.uwb_l[1,0] = l_uwb_msg.y
        self.check_uwb_l = True

    def uwb_r_callback(self, r_uwb_msg):
        pass
        current_time = rospy.Time.now()
        Ts = (current_time - self.time_ekf_update).to_sec()
        self.time_ekf_update = current_time

        w_r = self.vel['w']
        v_r = self.vel['v'] + w_r*self.uwb_distance/2
        self.vel_uwb_r = {'v_r': v_r,'w_r': w_r}
        u_r = np.array([[self.vel_uwb_r['v_r']], [self.vel_uwb_r['w_r']]])
        # rospy.loginfo("v_r = " + str(u_r[0,0]) + ", " + "w_r = " + str(u_r[1,0]))
        self.Q = np.diag([0.01, 0.01, 0.02]) ** 2
        self.R_UWB      = np.diag([0.31, 0.31]) ** 2
        # if self.check_uwb_r == True:
        z = np.array([[r_uwb_msg.x],[r_uwb_msg.y]])
        self.xEst_uwb_r, self.PEst_uwb_r = self.ekf_estimation_uwb(self.xEst_uwb_r, self.PEst_uwb_r, z, u_r, Ts)
        self.pose_uwb_r['x'] = self.xEst_uwb_r[0,0]
        self.pose_uwb_r['y'] = self.xEst_uwb_r[1,0]
        self.pose_uwb_r['th'] = self.xEst_uwb_r[2,0]
        self.uwb_r[0,0] = r_uwb_msg.x
        self.uwb_r[1,0] = r_uwb_msg.y
        self.check_uwb_r = True

    def spin(self):
        rospy.loginfo("[ROS] Start Node_UWB_Position")
        rate = rospy.Rate(50)   
        while not rospy.is_shutdown():
            self.update()            
            rate.sleep()

    #------------------------------------
    #----------------EKF (UWB) (asyn)----------------
    # robot system model --------------
    def robot_model_system(self,x, u,Ts):
        w = u[1,0]
        theta = x[2,0]
        F = np.array([[1.0, 0, 0],
                    [0, 1.0, 0],
                    [0, 0, 1.0]])  
        B = np.array([[Ts*math.cos(theta + 0.5*Ts*w),       0],
                    [Ts*math.sin(theta + 0.5*Ts*w),         0],
                    [0.0,                                   Ts]])
        x = F.dot(x) + B.dot(u)
        return x
    
    # Matrix Fk [3x3] -------------------------------
    def jacob_f(self,x, u,Ts):
        v       = u[0, 0]     # v - robot
        w       = u[1, 0]     # w - robot
        theta   = x[2,0]      # yaw
        #jF = 3x3
        jF = np.array([
            [1.0,   0.0,    -Ts * v * math.sin(theta + 0.5*Ts*w)],
            [0.0,   1.0,     Ts * v * math.cos(theta + 0.5*Ts*w)],
            [0.0,   0.0,                                    1.0]])
        return jF

    def jacob_h_uwb(self):
        # Jacobian of Observation Model
        jH = np.array([
            [1, 0, 0],
            [0, 1, 0]])
        return jH

    #------------EKF UWB + IMU----------------------------
    def ekf_estimation_uwb(self,xEst, PEst, z, u, Ts):
        #  Predict
        xPred = self.robot_model_system(xEst, u, Ts)
        jF = self.jacob_f(xPred, u, Ts)
        PPred = jF.dot(PEst).dot(jF.T) + self.Q 

        #  Update
        jH = self.jacob_h_uwb() #uwb + imu
        zPred = jH.dot(xPred)
        S = jH.dot(PPred).dot(jH.T) + self.R_UWB
        K = PPred.dot(jH.T).dot(np.linalg.inv(S))
    
        xEst = xPred + K.dot((z - zPred)) #
        PEst = (np.eye(len(xEst)) - K.dot(jH)).dot(PPred)

        # print("UWB: Kx= " + str(K[0,0]) + ", Ky=" + str(K[1,1]) + ", Kth=" + str(K[2,2]))
        return xEst, PEst 

def main():
    pos = UWB_Position()
    pos.spin()


                
if __name__ == '__main__':
    main()