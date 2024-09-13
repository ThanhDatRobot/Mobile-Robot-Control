#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist, Vector3
import math

pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
vel  = {'v': 0.0, 'w': 0.0}

# PID controller parameters
Kp_linear = 0.5
Ki_linear = 0.0
Kd_linear = 0.1

Kp_angular = 0.1
Ki_angular = 0.0
Kd_angular = 0.0

# Initialize PID errors
error_linear_prev = 0.0
integral_linear = 0.0

error_angular_prev = 0.0
integral_angular = 0.0

def pid_controller(xd, yd, xm, ym, theta, dt):
    global error_linear_prev, integral_linear, error_angular_prev, integral_angular

    # Calculate position error
    error_x = xd - xm
    error_y = yd - ym
    error_linear = np.sqrt(error_x**2 + error_y**2)

    # Calculate the angle to the target
    desired_angle = np.arctan2(error_y, error_x)
    error_angular = desired_angle - theta

    # Normalize angular error to the range [-pi, pi]
    error_angular = (error_angular + np.pi) % (2 * np.pi) - np.pi

    # PID for linear velocity
    integral_linear += error_linear * dt
    derivative_linear = (error_linear - error_linear_prev) / dt
    v = Kp_linear * error_linear + Ki_linear * integral_linear + Kd_linear * derivative_linear
    error_linear_prev = error_linear

    # PID for angular velocity
    integral_angular += error_angular * dt
    derivative_angular = (error_angular - error_angular_prev) / dt
    w = Kp_angular * error_angular + Ki_angular * integral_angular + Kd_angular * derivative_angular
    error_angular_prev = error_angular

    return v, w, error_x, error_y

def update_robot_position(v_R_ref, v_L_ref, r, b):
    v = r / 2 * (v_R_ref + v_L_ref)
    omega = r / (2 * b) * (v_R_ref - v_L_ref)
    return v, omega

radius = 0.5  # Bán kính của hình tròn
angular_velocity = 0.045  # Vận tốc góc của robot (radians per second)
dt = 0.01
total_time = 100

xm = 0.0
ym = 0.0
theta = 0.0

def circular_trajectory(radius, angular_velocity, time):
    xd = radius * np.cos(angular_velocity * time)
    yd = radius * np.sin(angular_velocity * time)
    return xd, yd

x_real = []
y_real = []
x_desired = []
y_desired = []
error_x_list = []
error_y_list = []

def subscriber_odom_callback(odom_data):
    global pose
    pose['x'] = odom_data.x
    pose['y'] = odom_data.y
    pose['yaw'] = (odom_data.z) * np.pi / 180

def vel_pub(v, w):
    vel_msg = Twist()
    vel_msg.linear.x = v
    vel_msg.angular.z = w
    pub_vel.publish(vel_msg)

def main():
    global pub_vel, previous_time, pose, time
    rospy.init_node('node_controller')
    pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/odom', Vector3, subscriber_odom_callback)
    rate = rospy.Rate(200)
    previous_time = rospy.Time.now()
    time = 0
    i = 0

    def shutdown_hook():
        rospy.loginfo("Ctrl+C pressed: Stopping the robot.")
        vel['v'] = 0
        vel['w'] = 0
        vel_pub(vel['v'], vel['w'])

    rospy.on_shutdown(shutdown_hook)
    
    while not rospy.is_shutdown():
        i += 1
        current_time = rospy.Time.now()
        time_duration = (current_time - previous_time).to_sec()
        if i >= 20:
            i = 0
            vel_pub(vel['v'], vel['w'])
        if time_duration >= 0.01:
            previous_time = current_time
            time += time_duration
            xd, yd = circular_trajectory(radius, angular_velocity, time)
            v, w, error_x, error_y = pid_controller(xd, yd, pose['x'], pose['y'], pose['yaw'], time_duration)
            
            error_x_list.append(error_x)
            error_y_list.append(error_y)
            
            vel['v'] = v
            vel['w'] = w
            
            x_real.append(pose['x'])
            y_real.append(pose['y'])
            x_desired.append(xd)
            y_desired.append(yd)
            
        rate.sleep()
    
    plt.figure()
    plt.plot(x_desired, y_desired, 'r--', label='Desired Path')
    plt.plot(x_real, y_real, 'b-', label='Real Path')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Robot Path Tracking')
    plt.legend()
    plt.gca().set_aspect('equal', 'box')

    plt.figure()
    plt.plot(error_x_list, 'r-', label='Error X')
    plt.plot(error_y_list, 'b-', label='Error Y')
    plt.xlabel('Time step')
    plt.ylabel('Error')
    plt.title('Tracking Errors')
    plt.legend()
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()

