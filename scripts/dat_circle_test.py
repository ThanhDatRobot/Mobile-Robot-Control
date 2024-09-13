#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist, Point, Vector3
import math


pose = {'x': -1.1, 'y': -1.3, 'yaw': 0.0}
vel  = {'v': 0.0 , 'w': 0.0}

def kinematic_controller(xd, xd_dot, yd, yd_dot, xm, ym, theta, zp, dt):
    # Declaration
    b = 0.155  # the half of the wheel shaft
    r = 0.05  # the radius of each wheel
    error_x = xd - xm
    error_y = yd - ym
    
    # Gamma
    gamma = 0.01  # Error when ep1 to zero
    exelon = 0.0001
    
    # Const
    d = np.sqrt(error_x**2 + error_y**2)  # distance to desired point
    C = 2  # d + 1.1 > to make sure that C>1
    L = 10
    
    # Transform of position
    T = np.array([[np.cos(theta), np.sin(theta)],
                  [-np.sin(theta), np.cos(theta)]])
    
    ep = np.dot(T, np.array([error_x, error_y]))  # error transform
    ep1 = ep[0]
    ep2 = ep[1]
    
    if (ep1 < 0):
        signed = -1
    else:
        signed = 1
        
    # Matrix to driving WMR
    h = np.array([[(ep2/b - 1) * (r/2), -(ep2/b + 1) * (r/2)],
                  [-ep1 * r / (2*b), ep1 * r / (2*b)]])
    
    if abs(ep1) >= gamma:
        # When ep1 >= gamma we use it
        hx = np.array([[-1/r, (-b - ep2) / (r * ep1)],
                       [-1/r, (b - ep2) / (r * ep1)]])
    else:
        # When ep1 < gamma we use it
        hx = np.array([[-1/r, (-b - ep2) / (r * signed * gamma)],
                       [-1/r, (b - ep2) / (r * signed * gamma)]])
    
    # Estimated disturbance observer and calculation for v2
    dp = zp + L * ep
    Kd = -1
    
    # Velocity
    # v1
    v1 = -np.dot(hx, np.dot(T, np.array([xd_dot, yd_dot])))
    
    # v2
    v2 = -np.dot(hx, (C * ep + Kd * dp))
    
    # Total velocity
    v = v1 + v2
    
     # Disturbance Observer
    zp_dot = -L * (zp + L * ep) - L * (np.dot(T, np.array([xd_dot, yd_dot])) + np.dot(h, v))
    
    # Update zp using Euler integration
    zp = zp + zp_dot * dt
    
    
    # Calculate for real Velocity Mobile Robot can do
    vmax = 4  # m/s => rad/s 0.02 => 4 rad/s 
    if abs(v[0]) - abs(v[1]) > 0:
        if abs(v[0]) > vmax:
            v[1] = v[1] * vmax / v[0]
            v[0] = vmax
    else:
        if abs(v[1]) > vmax:
            v[0] = v[0] * vmax / v[1]
            v[1] = vmax
    
    gain = 0.25
    v_R_ref = v[0] * gain
    v_L_ref = v[1] * gain

    
    return v_R_ref, v_L_ref, error_x, error_y, zp, zp_dot

# Hàm cập nhật vị trí của robot dựa trên tốc độ tham chiếu của các bánh xe
def update_robot_position(v_R_ref, v_L_ref, r, b):
    # Tính toán vận tốc tuyến tính và vận tốc góc của robot
    v = r / 2 * (v_R_ref + v_L_ref)
    omega = r / (2 * b) * (v_R_ref - v_L_ref)
    return v, omega

# Tham số của quỹ đạo
radius = 1  # Bán kính của hình tròn
angular_velocity = 0.01  # Vận tốc góc của robot (radians per second)

# Thời gian delta và thời gian chạy
dt = 0.01
total_time = 100

# Vị trí ban đầu của robot
xm = 0.0
ym = 0.0
theta = 0.0
zp = np.array([0.0, 0.0])

def square_trajectory(side_length, speed, time, pre_time, current_side):
    t_per_side = side_length / speed
    time_in_side = time - pre_time

    if current_side == 1:
        xd = -1 + speed * time_in_side
        yd = -1
        xd_dot = speed
        yd_dot = 0
        if (xd >1):
            xd = 1
    elif current_side == 2:
        xd = 1
        yd = -1 + speed * time_in_side
        xd_dot = 0
        yd_dot = speed
        if (yd >1):
            yd = 1
    elif current_side == 3:
        xd = 1 - speed * time_in_side
        yd = 1
        xd_dot = -speed
        yd_dot = 0
        if (xd <-1):
            xd = -1
    elif current_side == 4:
        xd = -1
        yd = 1 - speed * time_in_side
        xd_dot = 0
        yd_dot = -speed
        if (yd <-1):
            yd = -1

    return xd, yd, xd_dot, yd_dot

# Tham số của quỹ đạo
side_length = 2.0  # Chiều dài cạnh của hình vuông
speed = 0.05  # Vận tốc của robot trên mỗi cạnh (m/s)

# Lưu trữ các giá trị để vẽ đồ thị
x_real = []
y_real = []
x_desired = []
y_desired = []
error_x_list = []
error_y_list = []
v_R_ref_list = []
v_L_ref_list = []

def subscriber_odom_callback(odom_data):
    global pose
    pose['x'] = odom_data.x
    pose['y'] = odom_data.y
    #pose['yaw'] = (odom_data.z)
    pose['yaw'] = (odom_data.z)*np.pi/180
    x = pose['x']
    y = pose['y']
    yaw = pose['yaw']
    #rospy.loginfo(f'Received x: {x:.5f}, y: {y:.5f}, yaw: {yaw:.5f}')

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
    pre_time = 0
    i = 0
    s = 0
    zp = np.array([0.0, 0.0])
    current_side = 1


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
        if i >= 20:  # ~ 100ms = 10Hz
            i = 0
            vel_pub(vel['v'], vel['w'])
        if time_duration >= 0.01:
            previous_time = current_time
            time += time_duration
            xd, yd, xd_dot, yd_dot = square_trajectory(side_length, speed, time, pre_time, current_side)
            v_R_ref, v_L_ref, error_x, error_y, zp, zp_dot = kinematic_controller(xd, xd_dot, yd, yd_dot, pose['x'], pose['y'], pose['yaw'], zp, time_duration)
            
            if (current_side == 1 and pose['x'] >= -1 + side_length - 0.05):
               current_side =2
               pre_time = time
            if (current_side == 2 and pose['y'] >= -1 + side_length - 0.05):
               current_side =3
               pre_time = time
            if (current_side ==3 and pose['x'] <= -1 + 0.05):
               current_side =4 
               pre_time = time
            if (current_side == 4 and pose['y'] <= -1 + 0.05): 
               current_side =1
               pre_time = time

            error_x_list.append(error_x)
            error_y_list.append(error_y)
            
            vel['v'], vel['w'] = update_robot_position(v_R_ref, v_L_ref, 0.05, 0.155)
			
            # Store positions for plotting
            x_real.append(pose['x'])
            y_real.append(pose['y'])
            x_desired.append(xd)
            y_desired.append(yd)
            v_R_ref_list.append(v_R_ref*0.05)
            v_L_ref_list.append(v_L_ref*0.05)
            
        rate.sleep()
    
    plt.rcParams.update({'font.size': 16})  # Tăng cỡ chữ tổng thể
    fig_size = (8, 8)  # Kích thước cố định cho các figure, đảm bảo hình vuông
    plt.figure()
    plt.plot(x_desired, y_desired, 'r--', label='Desired')
    plt.plot(x_real, y_real, 'b-', label='Robot Tracking')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Tracking')
    plt.legend(loc='upper right')  # Chú thích ở góc trên bên phải
    plt.gca().set_aspect('equal', 'box')  # Đảm bảo tỷ lệ chính xác giữa các trục

    plt.figure()
    plt.plot(error_x_list, 'r-', label='Error X')
    plt.plot(error_y_list, 'b-', label='Error Y')
    plt.xlabel('Time')
    plt.ylabel('Error')
    plt.title('Error Tracking')
    plt.legend(loc='upper right')  # Chú thích ở góc trên bên phải
    plt.tight_layout()

# Plot the reference velocities

    # Vận tốc
    fig3, (ax3, ax4) = plt.subplots(2, 1, figsize=fig_size, sharex=True)
    ax3.plot(np.array(range(len(v_R_ref_list))) * 0.01, v_R_ref_list, 'r-', label='v_R_ref')  # Nhân trục x với 0.01
    ax4.plot(np.array(range(len(v_L_ref_list))) * 0.01, v_L_ref_list, 'b-', label='v_L_ref')  # Nhân trục x với 0.01
    ax3.set_xlim([0, 160])
    ax4.set_xlim([0, 160])
    ax3.set_ylabel('Velocity (m/s)', fontsize=18)  # Tăng kích thước ylabel
    ax4.set_ylabel('Velocity (m/s)', fontsize=18)  # Tăng kích thước ylabel
    ax4.set_xlabel('Time(s)', fontsize=18)  # Tăng kích thước xlabel
    ax3.set_title('Right Wheel Velocity', fontsize=20)  # Tăng kích thước title
    ax4.set_title('Left Wheel Velocity', fontsize=20)  # Tăng kích thước title
    ax3.legend(loc='upper right')
    ax4.legend(loc='upper right')
    #ax3.grid(True)
    #ax4.grid(True)

    # Tăng kích thước chữ số ở trục x và y
    ax3.tick_params(axis='both', which='major', labelsize=16)
    ax4.tick_params(axis='both', which='major', labelsize=16)

    plt.show()
    
  

if __name__ == '__main__':
    main()
