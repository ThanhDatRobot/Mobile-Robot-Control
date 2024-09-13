#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist, Vector3
import math

class KinematicController:
    def __init__(self):
        # Khởi tạo node
        rospy.init_node('node_controller')
        
        # Publisher và Subscriber
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Vector3, self.subscriber_odom_callback)
        
        # Các tham số
        self.pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.vel = {'v': 0.0, 'w': 0.0}
        self.zp = np.array([0.0, 0.0])
        
        # Tham số quỹ đạo
        self.radius = 0.5
        self.angular_velocity = 0.03
        
        # Tham số thời gian
        self.dt = 0.01
        self.total_time = 100
        self.time = 0
        
        # Danh sách lỗi để vẽ đồ thị
        self.error_x_list = []
        self.error_y_list = []
        self.x_real = []
        self.y_real = []
        self.x_desired = []
        self.y_desired = []
        
        # Cờ để xác định xem có dữ liệu mới từ subscriber hay không
        self.new_data_received = False
        
        # ROS Timer cho vòng lặp điều khiển
        rospy.Timer(rospy.Duration(self.dt), self.control_loop)
        
    def subscriber_odom_callback(self, odom_data):
        self.pose['x'] = odom_data.x
        self.pose['y'] = odom_data.y
        self.pose['yaw'] = (odom_data.z) * np.pi / 180
        rospy.loginfo(f'Nhận được x: {self.pose["x"]:.5f}, y: {self.pose["y"]:.5f}, yaw: {self.pose["yaw"]:.5f}')
        self.new_data_received = True  # Đánh dấu đã nhận được dữ liệu mới
    
    def vel_pub(self, v, w):
        vel_msg = Twist()
        vel_msg.linear.x = v
        vel_msg.angular.z = w
        self.pub_vel.publish(vel_msg)
    
    def circular_trajectory(self, time):
        xd = self.radius * np.cos(self.angular_velocity * time)
        yd = self.radius * np.sin(self.angular_velocity * time)
        xd_dot = -self.radius * self.angular_velocity * np.sin(self.angular_velocity * time)
        yd_dot = self.radius * self.angular_velocity * np.cos(self.angular_velocity * time)
        return xd, yd, xd_dot, yd_dot
    
    def kinematic_controller(self, xd, xd_dot, yd, yd_dot, xm, ym, theta, zp, dt):
        b = 0.161
        r = 0.05
        error_x = xd - xm
        error_y = yd - ym
        
        gamma = 0.01
        exelon = 0.0001
        d = np.sqrt(error_x**2 + error_y**2)
        C = 5
        L = 2
        
        T = np.array([[np.cos(theta), np.sin(theta)],
                      [-np.sin(theta), np.cos(theta)]])
        
        ep = np.dot(T, np.array([error_x, error_y]))
        ep1 = ep[0]
        ep2 = ep[1]
        
        signed = -1 nếu ep1 < 0 else 1
        
        h = np.array([[(ep2/b - 1) * (r/2), -(ep2/b + 1) * (r/2)],
                      [-ep1 * r / (2*b), ep1 * r / (2*b)]])
        
        if abs(ep1) >= gamma:
            hx = np.array([[-1/r, (-b - ep2) / (r * ep1)],
                           [-1/r, (b - ep2) / (r * ep1)]])
        else:
            hx = np.array([[-1/r, (-b - ep2) / (r * signed * gamma)],
                           [-1/r, (b - ep2) / (r * signed * gamma)]])
        
        dp = zp + L * ep
        Kd = -1
        
        v1 = -np.dot(hx, np.dot(T, np.array([xd_dot, yd_dot])))
        v2 = -np.dot(hx, (C * ep + Kd * dp))
        v = v1 + v2
        
        zp_dot = -L * (zp + L * ep) - L * (np.dot(T, np.array([xd_dot, yd_dot])) + np.dot(h, v))
        zp = zp + zp_dot * dt
        
        vmax = 4
        if abs(v[0]) - abs(v[1]) > 0:
            if abs(v[0]) > vmax:
                v[1] = v[1] * vmax / v[0]
                v[0] = vmax
        else:
            if abs(v[1]) > vmax:
                v[0] = v[0] * vmax / v[1]
                v[1] = vmax
        
        gain = 4
        v_R_ref = v[0] * r * gain
        v_L_ref = v[1] * r * gain
        
        return v_R_ref, v_L_ref, error_x, error_y, zp, zp_dot
    
    def update_robot_position(self, v_R_ref, v_L_ref, r, b):
        v = r / 2 * (v_R_ref + v_L_ref)
        omega = r / (2 * b) * (v_R_ref - v_L_ref)
        return v, omega
    
    def control_loop(self, event):
        if self.new_data_received:  # Chỉ cập nhật khi có dữ liệu mới từ subscriber
            self.time += self.dt
            xd, yd, xd_dot, yd_dot = self.circular_trajectory(self.time)
            v_R_ref, v_L_ref, error_x, error_y, self.zp, _ = self.kinematic_controller(
                xd, xd_dot, yd, yd_dot, self.pose['x'], self.pose['y'], self.pose['yaw'], self.zp, self.dt)
            
            self.error_x_list.append(error_x)
            self.error_y_list.append(error_y)
            
            self.vel['v'], self.vel['w'] = self.update_robot_position(v_R_ref, v_L_ref, 0.05, 0.161)
            
            self.x_real.append(self.pose['x'])
            self.y_real.append(self.pose['y'])
            self.x_desired.append(xd)
            self.y_desired.append(yd)
            
            self.vel_pub(self.vel['v'], self.vel['w'])
            self.new_data_received = False  # Reset cờ dữ liệu mới
    
    def plot_results(self):
        plt.figure()
        ax1 = plt.subplot(2, 1, 1)
        ax1.plot(self.x_desired, self.y_desired, 'r--', label='Robot tracking')
        ax1.plot(self.x_real, self.y_real, 'b-', label='Desire tracking')
        ax1.set_xlabel('X')
        ax1.set_ylabel('Y')
        ax1.set_title('Tracking')
        ax1.legend()
        ax1.set_aspect('equal', 'box')  # Đảm bảo tỷ lệ chính xác giữa các trục
        
        ax2 = plt.subplot(2, 1, 2)
        ax2.plot(self.error_x_list, 'r-', label='Error X')
        ax2.plot(self.error_y_list, 'b-', label='Error Y')
        ax2.set_xlabel('time')
        ax2.set_ylabel('Error value')
        ax2.set_title('Error tracking')
        ax2.legend()
        
        plt.tight_layout()
        plt.show()

if __name__ == '__main__':
    try:
        controller = KinematicController()
        rospy.spin()
        controller.plot_results()
    except rospy.ROSInterruptException:
        pass

