#!/usr/bin/python3

msg = """
    EKF_UWB_IMU
    ------------------------------------
"""

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
from std_msgs.msg import String, Int8, Float32, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist, Vector3Stamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


# Covariance for EKF simulation
# Q = np.diag([
#     0.2,            # variance of location on x-axis
#     0.2,            # variance of location on y-axis
#     np.deg2rad(30)  # variance of yaw angle - 0.017453292519943295
# ]) ** 2             # predict state covariance
#Q1 = np.diag([0.4, 0.5])** 2    #v,w
# R_UWB              = np.diag([0.31, 0.31]) ** 2  # Observation x,y position covariance
#R_IMU              = 0.1


#--------------------------------------

class EKF_UWB_IMU_Position_Publisher:
    def __init__(self):
        rospy.init_node('Node_EKF_UWB_IMU_Publisher', anonymous=True)

        #Pub
        self.pose_uwb_imu_pub         = rospy.Publisher("/debug/pose_uwb_imu",            Point, queue_size=50)    #50Hz
        self.start_imu_pub = rospy.Publisher('/agv/start_imu', Bool, queue_size=10)
        self.reset_imu_pub = rospy.Publisher('/imu/reset_imu', Bool, queue_size=10)
        self.Pest_imu_pub  = rospy.Publisher("/debug/Pest_imu",            Point, queue_size=50)
        #Sub
        # rospy.Subscriber('/dwm1001/position', Point, self.uwb_position_callback)
        rospy.Subscriber('/debug/pose_uwb', Point, self.ekf_uwb_callback)
        rospy.Subscriber('/vel_pub', Twist, self.vel_callback)
        rospy.Subscriber("/imu/rpy/filtered", Vector3Stamped, callback=self.imu_callback)
        rospy.Subscriber("/debug/Pest_uwb", Point, callback=self.Pest_uwb_callback)

        self.vel = {'v':0.0 , 'w':0.0} #v robot (m/s)  && w robot (rad/s)

        #for debug
        self.time_uwb = rospy.Time.now()
        self.time_yaw = rospy.Time.now()
        self.time_vel = rospy.Time.now()
        self.last_time  = rospy.Time.now()

        #for predict EKF
        self.time_ekf_update = rospy.Time.now()
        self.PEst   = np.eye(3)*10 
        self.PEst_IMU   = np.eye(3)*10 
        self.xEst = np.zeros((3, 1))

        self.pose_uwb = {'x':0.0, 'y': 0.0, 'th': 0.0}
        self.pose_uwb_imu = {'x':0.0, 'y': 0.0, 'th': 0.0}

        #pre-updte
        self.xEst[0,0] = self.pose_uwb_imu['x']
        self.xEst[1,0] = self.pose_uwb_imu['y']
        self.xEst[2,0] = self.pose_uwb_imu['th']

        self.ready = False

        self.yaw_imu = 0.0
        self.yaw_tmp = 0.0
        self.yaw = 0.0

        self.Pest_yaw_uwb = 0.0
        self.Pest_yaw_imu = 0.0

        self.firsttime = True
        self.T = 0.0
        self.resetIMU = False

        self.K = 0.0    #do loi kalman
        self.drift = 0.01 #debug
        #R,Q
        self.R_IMU          = 0.1
        self.R_UWB_IMU      = np.diag([0.31, 0.31, 0.01]) ** 2

        self.Q = np.diag([  0.01, 0.01, 0.02]) ** 2 # OK predict state covariance
        self.Q1 = np.diag([0.4, 0.5])** 2    #v,w

        self.checkIMU = False
        self.checkUWB = False   #kiem tra lan nhau dau tien
        self.checkPestUWB = False
        
    def update(self):
        # pass
        if self.ready == True:
            pass       
            self.pub_pose_uwb_imu(self.pose_uwb_imu)
            self.pub_Pest_imu(self.PEst)

    def pub_pose_uwb_imu(self,pose_uwb_imu):
        pose_uwb_imu_msg = Point()
        pose_uwb_imu_msg.x = self.pose_uwb_imu['x']
        pose_uwb_imu_msg.y = self.pose_uwb_imu['y']
        pose_uwb_imu_msg.z = self.pose_uwb_imu['th']
        self.pose_uwb_imu_pub.publish(pose_uwb_imu_msg)

    def pub_Pest_imu(self, Pest_imu):
        Pest_imu_msg = Point()
        Pest_imu_msg.x = self.PEst[0,0]
        Pest_imu_msg.y = self.PEst[1,1]
        Pest_imu_msg.z = self.PEst[2,2]
        self.Pest_imu_pub.publish(Pest_imu_msg)
    
    def Pest_uwb_callback(self,Pest_uwb):
        pass
        self.Pest_yaw_uwb = Pest_uwb.z
        self.Pest_yaw_imu = self.PEst[2,2]
        # rospy.loginfo("uwb = " + str(self.Pest_yaw_uwb))
        # rospy.loginfo("imu = " + str(self.Pest_yaw_imu))
        self.checkPestUWB = True
    
    def vel_callback(self,vel_msg):
        pass
        self.vel['v'] = vel_msg.linear.x    # v robot (m/s)
        self.vel['w'] = vel_msg.angular.z   # w robot (rad/s)

    def imu_callback(self, imu_msg:Vector3Stamped):
        pass
        current_time = rospy.Time.now()
        Ts = (current_time - self.time_yaw).to_sec()   #deta_time
        self.time_yaw = current_time                   #update time
        self.yaw_imu = -float(imu_msg.vector.z)
        self.T = self.T + Ts
        if(self.firsttime == True):       #chi chay lan dau
            self.yaw_tmp = self.pose_uwb['z']
        if(self.resetIMU == True):
            self.yaw_tmp = self.pose_uwb['z']
            self.resetIMU = False
        if(self.firsttime == False and self.T > 1800):
            self.start_imu_pub.publish(Bool(False))
            self.reset_imu_pub.publish(Bool(True))
            self.start_imu_pub.publish(Bool(True))
            self.T = 0
            self.resetIMU = True
        self.yaw = self.yaw_imu + self.yaw_tmp
        if self.checkUWB == True and self.checkPestUWB == True:
            self.K = (self.Pest_yaw_imu)/(self.Pest_yaw_imu + self.Pest_yaw_uwb)
            rospy.loginfo("K = " + str(self.K))
            self.yaw = self.yaw + self.K*(self.pose_uwb['z'] - self.yaw)
            self.checkIMU = True
        self.firsttime = False
    
    def ekf_uwb_callback(self,ekf_uwb_msg):
        pass
        current_time = rospy.Time.now()
        Ts = (current_time - self.time_ekf_update).to_sec()   #deta_time
        self.time_ekf_update = current_time                   #update time
        self.pose_uwb['x'] = ekf_uwb_msg.x
        self.pose_uwb['y'] = ekf_uwb_msg.y
        self.pose_uwb['z'] = ekf_uwb_msg.z
        #
        u = np.array([[self.vel['v']], [self.vel['w']]])
        self.Q = np.diag([  0.01, 0.01, 0.02]) ** 2 # Day chac la tham so tot nhat
        self.R_UWB_IMU      = np.diag([0.31, 0.31, 0.001]) ** 2
        if self.vel['v'] == 0 and self.vel['w'] == 0:
            pass
            self.Q = np.diag([  0.01, 0.01, np.deg2rad(0.01)]) ** 2 # OK predict state covariance
            # self.R_UWB_IMU      = np.diag([0.31, 0.31, 10e4]) ** 2
            self.R_UWB_IMU      = np.diag([0.31, 0.31, 0.001]) ** 2
        else:
            pass
            self.R_UWB_IMU      = np.diag([0.31, 0.31, 0.001]) ** 2
        
        if self.checkUWB == True and self.checkIMU == True: #bo qua lan dau tien ko EKF Fusion
            z = np.array([[ekf_uwb_msg.x],[ekf_uwb_msg.y],[self.yaw]])
            self.xEst, self.PEst = self.ekf_estimation_uwb_imu(self.xEst, self.PEst, z, u, Ts) #PASSED: 100ms = 10Hz
                
            # Update to Odometry
            self.pose_uwb_imu['x'] = self.xEst[0,0]
            self.pose_uwb_imu['y'] = self.xEst[1,0]
            self.pose_uwb_imu['th'] = self.xEst[2,0]
            #
        self.ready = True
        self.checkUWB = True

    def spin(self): #ok
        rospy.loginfo("[ROS] Start Node_EKF_UWB_IMU_Publisher")
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

    def jacob_h_uwb_imu(self):
        # Jacobian of Observation Model
        jH = np.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]])
        return jH
    # H(UWB-IMU) [3x3] -----------------------

    #------------EKF UWB + IMU----------------------------
    def ekf_estimation_uwb_imu(self,xEst, PEst, z, u, Ts):
        #  Predict
        xPred = self.robot_model_system(xEst, u, Ts)
        jF = self.jacob_f(xPred, u, Ts)
        PPred = jF.dot(PEst).dot(jF.T) + self.Q 

        #  Update
        jH = self.jacob_h_uwb_imu() #uwb + imu
        zPred = jH.dot(xPred)
        S = jH.dot(PPred).dot(jH.T) + self.R_UWB_IMU
        K = PPred.dot(jH.T).dot(np.linalg.inv(S))
    
        xEst = xPred + K.dot((z - zPred)) #
        PEst = (np.eye(len(xEst)) - K.dot(jH)).dot(PPred).dot((np.eye(len(xEst)) - K.dot(jH)).T) + K.dot(self.R_UWB_IMU).dot(K.T)

        # print("UWB: Kx= " + str(K[0,0]) + ", Ky=" + str(K[1,1]) + ", Kth=" + str(K[2,2]))
        return xEst, PEst 

# -------------Main--------------------
def main():
    print(msg)
    ekf = EKF_UWB_IMU_Position_Publisher()
    ekf.spin()

if __name__ == '__main__':
    main()