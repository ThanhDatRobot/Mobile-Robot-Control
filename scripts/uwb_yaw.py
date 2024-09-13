#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Vector3, Vector3Stamped, Point
import random
from itertools import count
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure
from matplotlib.animation import FuncAnimation
from localizer_dwm1001.msg import Tag
from math import atan2, sin, cos

# plt.style.use('fivethirtyeight')
t = []

yaw_imu = []
yaw_uwb = []
yaw_ekf_uwb = []
yaw_DR = []
yaw_ekf_uwb_imu = []

class Yaw_Graph:
    yaw_tmp = {'imu': 0.0, 'uwb': 0.0, 'ekf_uwb': 0.0, 'DR': 0.0, 'ekf_uwb_imu': 0.0}
    pre_yaw = {'imu': 0.0, 'uwb': 0.0, 'ekf_uwb': 0.0, 'DR': 0.0, 'ekf_uwb_imu': 0.0}
    
    yaw1 = 0
    yaw2 = 0
    yaw3 = 0
    yaw4 = 0
    yaw5 = 0
    def __init__(self):
        rospy.init_node('Node_Yaw_UWB_Graph', anonymous=True)

        #Sub
        rospy.Subscriber("/imu/rpy/filtered", Vector3Stamped, callback=self.imu_callback)
        rospy.Subscriber("/dwm1001/position", Vector3, callback=self.uwb_callback)
        rospy.Subscriber("/debug/pose_uwb", Point, callback=self.uwb_ekf_callback)
        rospy.Subscriber("/debug/DR", Point, callback=self.DR_callback)
        rospy.Subscriber("/debug/pose_uwb_imu", Point, callback=self.uwb_imu_ekf_callback)

        #Graph
        self.fig = plt.figure(figsize=(15,12))
        self.fig.tight_layout()
        self.ax = self.fig.add_subplot(1, 1, 1)
        self.ax.set_ylim(-0.5, 0.5)

        # self.yaw_tmp = {'imu': 0.0, 'uwb': 0.0, 'ekf_uwb': 0.0, 'DR': 0.0, 'ekf_uwb_imu': 0.0}
        # self.pre_yaw = {'imu': 0.0, 'uwb': 0.0, 'ekf_uwb': 0.0, 'DR': 0.0, 'ekf_uwb_imu': 0.0}

        #for debug
        self.time_yaw = rospy.Time.now()

        # self.yaw1 = 0
        # self.yaw2 = 0
        # self.yaw3 = 0
        # self.yaw4 = 0
        # self.yaw5 = 0
        self.time = 20
        self.Ts = 0
        self.T = 0
    
    def imu_callback(self, imu_data: Vector3Stamped):
        # rospy.loginfo("yaw_imu = " + str(imu_data.vector.z))
        Yaw_Graph.yaw_tmp['imu'] = -float(imu_data.vector.z)
        delta = Yaw_Graph.yaw_tmp['imu'] - Yaw_Graph.pre_yaw['imu']
        delta = atan2(sin(delta), cos(delta))
        Yaw_Graph.pre_yaw['imu'] = Yaw_Graph.yaw_tmp['imu']
        Yaw_Graph.yaw1 += delta

    def uwb_callback(self, uwb_data: Vector3):
        # rospy.loginfo("yaw_uwb = " + str(uwb_data.z))
        Yaw_Graph.yaw_tmp['uwb'] = float(uwb_data.z)
        delta = Yaw_Graph.yaw_tmp['uwb'] - Yaw_Graph.pre_yaw['uwb']
        delta = atan2(sin(delta), cos(delta))
        Yaw_Graph.pre_yaw['uwb'] = Yaw_Graph.yaw_tmp['uwb']
        Yaw_Graph.yaw2 += delta

    def uwb_ekf_callback(self, uwb_ekf_data: Point):
        # rospy.loginfo("yaw_uwb_ekf = " + str(uwb_ekf_data.z))
        Yaw_Graph.yaw_tmp['ekf_uwb'] = float(uwb_ekf_data.z)
        delta = self.yaw_tmp['ekf_uwb'] - self.pre_yaw['ekf_uwb']
        delta = atan2(sin(delta), cos(delta))
        self.pre_yaw['ekf_uwb'] = Yaw_Graph.yaw_tmp['ekf_uwb']
        self.yaw3 += delta

    def DR_callback(self, DR_data: Point):
        # rospy.loginfo("yaw_DR = " + str(DR_data.z))
        Yaw_Graph.yaw_tmp['DR'] = float(DR_data.z)
        delta = self.yaw_tmp['DR'] - self.pre_yaw['DR']
        delta = atan2(sin(delta), cos(delta))
        self.pre_yaw['DR'] = Yaw_Graph.yaw_tmp['DR']
        self.yaw4 += delta

    def uwb_imu_ekf_callback(self, uwb_imu_ekf_data: Point):
        # rospy.loginfo("yaw_uwb_imu_ekf = " + str(uwb_imu_ekf_data.z))
        Yaw_Graph.yaw_tmp['ekf_uwb_imu'] = float(uwb_imu_ekf_data.z)
        delta = Yaw_Graph.yaw_tmp['ekf_uwb_imu'] - Yaw_Graph.pre_yaw['ekf_uwb_imu']
        delta = atan2(sin(delta), cos(delta))
        Yaw_Graph.pre_yaw['ekf_uwb_imu'] = Yaw_Graph.yaw_tmp['ekf_uwb_imu']
        Yaw_Graph.yaw5 += delta


    def animate(self, i):
        plt.cla()

        plt.suptitle("YAW UWB")

        t.append(self.T)

        current_time = rospy.Time.now()
        self.Ts = (current_time - self.time_yaw).to_sec()   #deta_time
        self.time_yaw = current_time                   #update time
        self.T = self.T + self.Ts

        yaw_imu.append(self.yaw1)
        yaw_uwb.append(self.yaw2)
        yaw_ekf_uwb.append(self.yaw3)
        yaw_DR.append(self.yaw4)
        yaw_ekf_uwb_imu.append(self.yaw5)

        # self.ax.plot(t, yaw_imu, linewidth=2, color='green', label="yaw_imu")
        self.ax.plot(t, yaw_uwb, linewidth=2, color='blue', label="yaw_uwb")
        self.ax.plot(t, yaw_ekf_uwb, linewidth=2, color='red', label="yaw_ekf_uwb")
        # self.ax.plot(t, yaw_DR, linewidth=1, color='black', label="yaw_DR")
        # self.ax.plot(t, yaw_ekf_uwb_imu, linewidth=2, color='purple', label="yaw_ekf_uwb_imu")

        self.ax.set_xlabel('Time(s)')
        self.ax.set_ylabel('Yaw(radian)')
        self.ax.grid(visible=True, which='major', axis='both')
        self.ax.legend(loc="upper right")
        self.ax.set_xlim(xmin=0)
        
    def show(self): #ok
        rospy.loginfo("[ROS] Start Node_Yaw_Graph")
        self.ani = FuncAnimation(self.fig, self.animate, interval=self.time)       
        plt.show()   

if __name__ == '__main__':
    yaw_graph = Yaw_Graph()
    yaw_graph.show()
    rospy.spin()
