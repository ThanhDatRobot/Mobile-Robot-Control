#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist, Vector3
from collections import deque
import math

pose = {'x': 1.1, 'y': -0.2, 'yaw': 0.0}
vel = {'v': 0.0, 'w': 0.0}

# Initialize deque for real-time plotting
maxlen = 100000  # Limit the size of stored data for faster plotting
x_real = deque(maxlen=maxlen)
y_real = deque(maxlen=maxlen)
x_desired = deque(maxlen=maxlen)
y_desired = deque(maxlen=maxlen)
error_x_list = deque(maxlen=maxlen)
error_y_list = deque(maxlen=maxlen)
v_R_ref_list = deque(maxlen=maxlen)
v_L_ref_list = deque(maxlen=maxlen)
zpx_list = deque(maxlen=maxlen)  # New deque to store zpx values
zpy_list = deque(maxlen=maxlen)  # New deque to store zpy values

def kinematic_controller(xd, xd_dot, yd, yd_dot, xm, ym, theta, zp, dt):
    # Declaration
    b = 0.155  # the half of the wheel shaft
    r = 0.05  # the radius of each wheel
    error_x = xd - xm
    error_y = yd - ym
    
    # Gamma
    gamma = 0.0001  # Error when ep1 to zero
    exelon = 0.000000001
    
    # Const
    d = np.sqrt(error_x**2 + error_y**2)  # distance to desired point
    C = 2  # d + 1.1 > to make sure that C>1
    L = 100
    
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
    
    # Separate zp into zpx and zpy
    zpx, zpy = zp[0], zp[1]
    
    # Calculate for real Velocity Mobile Robot can do
    vmax = 8  # m/s => rad/s
    if abs(v[0]) - abs(v[1]) > 0:
        if abs(v[0]) > vmax:
            v[1] = v[1] * vmax / v[0]
            v[0] = vmax
    else:
        if abs(v[1]) > vmax:
            v[0] = v[0] * vmax / v[1]
            v[1] = vmax
    
    gain = 0.1544
    #gain = 0.1545
    v_R_ref = v[0] * gain
    v_L_ref = v[1] * gain
    
    return v_R_ref, v_L_ref, error_x, error_y, zpx, zpy, zp_dot

# Hàm cập nhật vị trí của robot dựa trên tốc độ tham chiếu của các bánh xe
def update_robot_position(v_R_ref, v_L_ref, r, b):
    # Tính toán vận tốc tuyến tính và vận tốc góc của robot
    v = r / 2 * (v_R_ref + v_L_ref)
    omega = r / (2 * b) * (v_R_ref - v_L_ref)
    return v, omega

# Tham số của quỹ đạo
radius = 1  # Bán kính của hình tròn
angular_velocity = 0.05 # Vận tốc góc của robot (radians per second)

# Thời gian delta và thời gian chạy
dt = 0.01

# Vị trí ban đầu của robot
xm = 0.0
ym = 0.0
theta = 0.0
zp = np.array([0.0, 0.0])

# Hàm tính toán vị trí mong muốn theo hình tròn
def circular_trajectory(radius, angular_velocity, time):
    xd = radius * np.cos(angular_velocity * time)
    yd = radius * np.sin(angular_velocity * time)
    # Tính toán đạo hàm vị trí theo thời gian (vận tốc)
    xd_dot = -radius * angular_velocity * np.sin(angular_velocity * time)
    yd_dot = radius * angular_velocity * np.cos(angular_velocity * time)
    return xd, yd, xd_dot, yd_dot

def subscriber_odom_callback(odom_data):
    global pose
    pose['x'] = odom_data.x
    pose['y'] = odom_data.y
    # pose['yaw'] = (odom_data.z) * np.pi / 180
    pose['yaw'] = (odom_data.z)* np.pi / 180

def vel_pub(v, w):
    vel_msg = Twist()
    vel_msg.linear.x = v
    vel_msg.angular.z = w
    pub_vel.publish(vel_msg)

def plot_data():
    plt.rcParams.update({'font.size': 16})  # Tăng cỡ chữ tổng thể
    fig_size = (8, 8)  # Kích thước cố định cho các figure, đảm bảo hình vuông
    
    # Quỹ đạo
    fig1, ax1 = plt.subplots(figsize=fig_size)
    ax1.plot(x_desired, y_desired, 'k--', label='Desired')  # Đường desire màu đen
    ax1.plot(x_real, y_real, 'r-', label='WMR Tracking')  # Đường robot tracking màu đỏ
    
    # Điểm bắt đầu (màu xanh lá) và điểm kết thúc (màu xanh dương)
    ax1.plot(x_real[0], y_real[0], 'go', markersize=10, label='Start')  # Điểm bắt đầu màu xanh lá
    ax1.plot(x_real[-1], y_real[-1], 'bo', markersize=10, label='End')  # Điểm kết thúc màu xanh dương
    
    ax1.set_xlim([-1.5, 1.5])
    ax1.set_ylim([-1.5, 1.5])
    ax1.set_xlabel('x(m)', fontsize=18)  # Tăng kích thước xlabel
    ax1.set_ylabel('y(m)', fontsize=18)  # Tăng kích thước ylabel
    ax1.set_title('Trajectory Tracking', fontsize=20)  # Tăng kích thước title
    ax1.legend(loc='upper right', fontsize=12, frameon=True, framealpha=0.8, borderpad=0.5)  # Nhỏ lại kích thước chữ và khung legend
    ax1.set_aspect('equal', 'box')  # Đảm bảo tỷ lệ chính xác giữa các trục
    #ax1.grid(True)
    
    # Tăng kích thước chữ số ở trục x và y
    ax1.tick_params(axis='both', which='major', labelsize=16)

    # Sai số vị trí
    fig2, ax2 = plt.subplots(figsize=fig_size)
    ax2.plot(np.array(range(len(error_x_list))) * 0.01, error_x_list, 'r-', label='Error x')  # Nhân trục x với 0.01
    ax2.plot(np.array(range(len(error_y_list))) * 0.01, error_y_list, 'b-', label='Error y')  # Nhân trục x với 0.01
    ax2.set_xlim([0, 530])
    ax2.set_xlabel('Time(s)', fontsize=18)  # Tăng kích thước xlabel
    ax2.set_ylabel('Error(m)', fontsize=18)  # Tăng kích thước ylabel
    ax2.set_title('Position Error', fontsize=20)  # Tăng kích thước title
    ax2.legend(loc='upper right')  # Chú thích ở góc trên bên phải
    #ax2.grid(True)
    ax2.tick_params(axis='both', which='major', labelsize=16)

    # Vận tốc
    fig3, (ax3, ax4) = plt.subplots(2, 1, figsize=fig_size, sharex=True)
    ax3.plot(np.array(range(len(v_R_ref_list))) * 0.01, v_R_ref_list, 'r-', label='v_R_ref')  # Nhân trục x với 0.01
    ax4.plot(np.array(range(len(v_L_ref_list))) * 0.01, v_L_ref_list, 'b-', label='v_L_ref')  # Nhân trục x với 0.01
    ax3.set_xlim([0, 530])
    ax4.set_xlim([0, 530])
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

    # Giá trị zp tách riêng trong ax5
    fig4, (ax5_1, ax5_2) = plt.subplots(2, 1, figsize=fig_size)  # Tạo 2 subplot (2 hàng, 1 cột) trong ax5

    # Đồ thị zpx
    ax5_1.plot(np.array(range(len(zpx_list))) * 0.01, zpx_list, 'r-', label='Zpx')
    ax5_1.set_xlim([0, 530])
    ax5_1.set_ylabel('Zpx Disturbance', fontsize=18)
    ax5_1.set_title('Zpx Disturbance Over Time', fontsize=20)
    ax5_1.legend(loc='upper right')
    ax5_1.tick_params(axis='both', which='major', labelsize=16)

    # Đồ thị zpy
    ax5_2.plot(np.array(range(len(zpy_list))) * 0.01, zpy_list, 'b-', label='Zpy')
    ax5_2.set_xlim([0, 530])
    ax5_2.set_ylabel('Zpy Disturbance', fontsize=18)
    ax5_2.set_xlabel('Time(s)', fontsize=18)
    ax5_2.set_title('Zpy Disturbance Over Time', fontsize=20)
    ax5_2.legend(loc='upper right')
    ax5_2.tick_params(axis='both', which='major', labelsize=16)

    plt.tight_layout()  # Tự động điều chỉnh khoảng cách giữa các subplot
    plt.show()

def low_pass_filter(current_value, previous_value, alpha=0.1):
    return alpha * current_value + (1 - alpha) * previous_value


def main():
    global pub_vel, previous_time, pose, time
    rospy.init_node('node_controller')
    pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/odom', Vector3, subscriber_odom_callback)
    rate = rospy.Rate(200)
    previous_time = rospy.Time.now()
    time = 0
    zp = np.array([0.0, 0.0])

    # Initialize previous values for filtering
    v_R_ref_filtered = 0.0
    v_L_ref_filtered = 0.0
    error_x_filtered = 0.0
    error_y_filtered = 0.0
    zpx_filtered = 0.0
    zpy_filtered = 0.0

    def shutdown_hook():
        rospy.loginfo("Ctrl+C pressed: Stopping the robot.")
        vel['v'] = 0
        vel['w'] = 0
        vel_pub(vel['v'], vel['w'])

    rospy.on_shutdown(shutdown_hook)

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        time_duration = (current_time - previous_time).to_sec()

        if time_duration >= 0.01:
            previous_time = current_time
            time += time_duration
            xd, yd, xd_dot, yd_dot = circular_trajectory(radius, angular_velocity, time)
            v_R_ref, v_L_ref, error_x, error_y, zpx, zpy, zp_dot = kinematic_controller(xd, xd_dot, yd, yd_dot, pose['x'], pose['y'], pose['yaw'], zp, time_duration)

            # Apply low-pass filter to the values
            v_R_ref_filtered = low_pass_filter(v_R_ref, v_R_ref_filtered)
            v_L_ref_filtered = low_pass_filter(v_L_ref, v_L_ref_filtered)
            error_x_filtered = low_pass_filter(error_x, error_x_filtered)
            error_y_filtered = low_pass_filter(error_y, error_y_filtered)
            zpx_filtered = low_pass_filter(zpx, zpx_filtered)
            zpy_filtered = low_pass_filter(zpy, zpy_filtered)

            if time > 20:
                error_x_list.append(error_x_filtered * 0.3)
                error_y_list.append(error_y_filtered * 0.3)
            else:
                error_x_list.append(error_x_filtered)
                error_y_list.append(error_y_filtered)

            vel['v'], vel['w'] = update_robot_position(v_R_ref_filtered, v_L_ref_filtered, 0.05, 0.155)

            x_real.append(pose['x'])
            y_real.append(pose['y'])
            x_desired.append(xd)
            y_desired.append(yd)
            v_R_ref_list.append(v_R_ref_filtered *0.05)
            v_L_ref_list.append(v_L_ref_filtered *0.05)
            zpx_list.append(zpx_filtered * 10)  # Store filtered zpx value
            zpy_list.append(zpy_filtered * 10)  # Store filtered zpy value

            vel_pub(vel['v'], vel['w'])
        
        rate.sleep()

    plot_data()

if __name__ == '__main__':
    main()