#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Vector3, Vector3Stamped, Point, PoseWithCovarianceStamped
import random
from itertools import count
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure
from matplotlib.animation import FuncAnimation
from localizer_dwm1001.msg import Tag
from nav_msgs.msg import Odometry
from math import atan2, sin, cos
import math
from math import sin, cos, pi

plt.style.use('fivethirtyeight')

x_ = {'odom': [], 'imu': [], 'uwb': [], 'ekf_uwb': [], 'ekf_uwb_imu':[], 'DR': [], 'AMCL': []}
y_ = {'odom': [], 'imu': [], 'uwb': [], 'ekf_uwb': [], 'ekf_uwb_imu':[], 'DR': [], 'AMCL': []}

class XY_Graph:
    x = {'odom': 0.0,'imu': 0.0, 'uwb': 0.0, 'ekf_uwb': 0.0, 'ekf_uwb_imu':0.0, 'DR': 0.0, 'AMCL': 0.0}
    y = {'odom': 0.0,'imu': 0.0, 'uwb': 0.0, 'ekf_uwb': 0.0, 'ekf_uwb_imu':0.0, 'DR': 0.0, 'AMCL': 0.0}
    
    def __init__(self):
        rospy.init_node('Node_XY_Graph', anonymous=True)

        #Sub
        rospy.Subscriber("/odom", Odometry, callback=self.odom_callback)
        rospy.Subscriber("/debug/pose_imu", Point, callback=self.imu_callback)
        rospy.Subscriber("/dwm1001/position", Vector3, callback=self.uwb_callback)
        rospy.Subscriber("/debug/pose_uwb", Point, callback=self.uwb_ekf_callback)
        rospy.Subscriber("/debug/DR", Point, callback=self.DR_callback)
        rospy.Subscriber("/debug/pose_uwb_imu", Point, callback=self.uwb_imu_ekf_callback)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback=self.amcl_callback)

        #Graph
        self.fig = plt.figure(figsize=(15,12))
        self.fig.tight_layout()
        self.ax = self.fig.add_subplot(1, 1, 1)

        # self.x = {'imu': 0.0, 'uwb': 0.0, 'ekf_uwb': 0.0, 'ekf_uwb_imu':0.0, 'DR': 0.0, 'AMCL': 0.0}
        # self.y = {'imu': 0.0, 'uwb': 0.0, 'ekf_uwb': 0.0, 'ekf_uwb_imu':0.0, 'DR': 0.0, 'AMCL': 0.0}

        #for debug
        self.time_yaw = rospy.Time.now()

        self.time = 20
        self.Ts = 0
        self.T = 0
        self.yaw_imu = 0.0
    
    def odom_callback(self, odom_data: Odometry):
        XY_Graph.x['odom'] = float(odom_data.pose.pose.position.x)
        XY_Graph.y['odom'] = float(odom_data.pose.pose.position.y)

    def imu_callback(self, imu_data: Point):
        XY_Graph.x['imu'] = float(imu_data.x)
        XY_Graph.y['imu'] = float(imu_data.y) 

    def uwb_callback(self, uwb_data: Vector3):
        XY_Graph.x['uwb'] = float(uwb_data.x)
        XY_Graph.y['uwb'] = float(uwb_data.y)

    def uwb_ekf_callback(self, uwb_ekf_data: Point):
        XY_Graph.x['ekf_uwb'] = float(uwb_ekf_data.x)
        XY_Graph.y['ekf_uwb'] = float(uwb_ekf_data.y)

    def DR_callback(self, DR_data: Point):
        XY_Graph.x['DR'] = float(DR_data.x)
        XY_Graph.y['DR'] = float(DR_data.y)

    def uwb_imu_ekf_callback(self, uwb_imu_ekf_data: Point):
        XY_Graph.x['ekf_uwb_imu'] = float(uwb_imu_ekf_data.x)
        XY_Graph.y['ekf_uwb_imu'] = float(uwb_imu_ekf_data.y)

    def amcl_callback(self, amcl_data: PoseWithCovarianceStamped):
    # rospy.loginfo("(" + str(uwb_data.x) + ", " + str(uwb_data.y) + ")")
        XY_Graph.x['AMCL'] = float(amcl_data.pose.pose.position.x)
        XY_Graph.y['AMCL'] = float(amcl_data.pose.pose.position.y)

    def animate(self, i):
        plt.cla()

        plt.suptitle("XY graph")

        x_['odom'].append(XY_Graph.x['odom'])
        x_['imu'].append(XY_Graph.x['imu'])
        x_['uwb'].append(XY_Graph.x['uwb'])
        x_['ekf_uwb'].append(XY_Graph.x['ekf_uwb'])
        x_['DR'].append(XY_Graph.x['DR'])
        x_['ekf_uwb_imu'].append(XY_Graph.x['ekf_uwb_imu'])
        x_['AMCL'].append(XY_Graph.x['AMCL'])
        
        y_['odom'].append(XY_Graph.y['odom'])
        y_['imu'].append(XY_Graph.y['imu'])
        y_['uwb'].append(XY_Graph.y['uwb'])
        y_['ekf_uwb'].append(XY_Graph.y['ekf_uwb'])
        y_['DR'].append(XY_Graph.y['DR'])
        y_['ekf_uwb_imu'].append(XY_Graph.y['ekf_uwb_imu'])
        y_['AMCL'].append(XY_Graph.y['AMCL'])

        self.ax.plot(x_['odom'], y_['odom'], linewidth=3, color='yellow', label="xy_odom")
        self.ax.plot(x_['imu'], y_['imu'], linewidth=3, color='green', label="xy_imu")
        self.ax.plot(x_['uwb'], y_['uwb'], linewidth=3, color='blue', label="xy_uwb")
        self.ax.plot(x_['ekf_uwb'], y_['ekf_uwb'], linewidth=3, color='red', label="xy_ekf_uwb")
        self.ax.plot(x_['DR'], y_['DR'], linewidth=3, color='black', label="xy_DR")
        self.ax.plot(x_['ekf_uwb_imu'], y_['ekf_uwb_imu'], linewidth=3, color='purple', label="xy_ekf_uwb_imu")
        # self.ax.plot(x['AMCL'], y['AMCL'], linewidth=3, color='orange', label="xy_AMCL")

        self.ax.set_xlabel('X(m)')
        self.ax.set_ylabel('Y(m)')
        self.ax.grid(visible=True, which='major', axis='both')
        self.ax.legend(loc="upper right")
        
    def show(self): #ok
        rospy.loginfo("[ROS] Start Node_XY_Graph")
        self.ani = FuncAnimation(self.fig, self.animate, interval=self.time)       
        plt.show()   

if __name__ == '__main__':
    xy_graph = XY_Graph()
    xy_graph.show()
    rospy.spin()