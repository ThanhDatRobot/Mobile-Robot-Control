#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist, Vector3
from collections import deque
import math

pose = {'x': -1.1, 'y': -1.3, 'yaw': 0.0}
vel  = {'v': 0.0 , 'w': 0.0}

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
    gamma = 0.001  # Error when ep1 to zero
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
    vmax = 8  # m/s => rad/s 0.02 => 4 rad/s 
    if abs(v[0]) - abs(v[1]) > 0:
        if abs(v[0]) > vmax:
            v[1] = v[1] * vmax / v[0]
            v[0] = vmax
    else:
        if abs(v[1]) > vmax:
            v[0] = v[0] * vmax / v[1]
            v[1] = vmax
    
    gain = 0.115
    v_R_ref = v[0] * gain
    v_L_ref = v[1] * gain
    
    return v_R_ref, v_L_ref, error_x, error_y, zpx, zpy, zp_dot

# Hàm cập nhật vị trí của robot dựa trên tốc độ tham chiếu của các bánh xe
def update_robot_position(v_R_ref, v_L_ref, r, b):
    # Tính toán vận tốc tuyến tính và vận tốc góc của robot
    v = r / 2 * (v_R_ref + v_L_ref)
    omega = r / (2 * b) * (v_R_ref - v_L_ref)
    return v, omega


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
speed = 0.035  # Vận tốc của robot trên mỗi cạnh (m/s)

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
    #rospy.loginfo(f'Received x: {x:.5f}, y: {y:.5f}, yaw: {yaw:.5f}')

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
    
    ax1.set_xlim([-2, 2]) #### chỉnh ở đây
    ax1.set_ylim([-2, 2]) #### chỉnh ở đây
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
    ax2.set_xlim([0,180])
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
    ax3.set_xlim([0, 180])
    ax4.set_xlim([0, 180])
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
    ax5_1.set_xlim([0, 180])
    ax5_1.set_ylabel('Zpx Disturbance', fontsize=18)
    ax5_1.set_title('Zpx Disturbance Over Time', fontsize=20)
    ax5_1.legend(loc='upper right')
    ax5_1.tick_params(axis='both', which='major', labelsize=16)

    # Đồ thị zpy
    ax5_2.plot(np.array(range(len(zpy_list))) * 0.01, zpy_list, 'b-', label='Zpy')
    ax5_2.set_xlim([0, 180])
    ax5_2.set_ylabel('Zpy Disturbance', fontsize=18)
    ax5_2.set_xlabel('Time(s)', fontsize=18)
    ax5_2.set_title('Zpy Disturbance Over Time', fontsize=20)
    ax5_2.legend(loc='upper right')
    ax5_2.tick_params(axis='both', which='major', labelsize=16)

    plt.tight_layout()  # Tự động điều chỉnh khoảng cách giữa các subplot
    plt.show()

def low_pass_filter(current_value, previous_value, alpha=1):
    return alpha * current_value + (1 - alpha) * previous_value
### chỗ này alpha => 1 đáp ứng nhanh ### dễ bị giật
### chỗ này alpha =>0 sự thay đổi chậm ### robot ít bị giật
    
# Thời gian delta và thời gian chạy
dt = 0.01 ### giảm cái này thì tăng cái độ phản hồi, tăng tốc độ lấy mẫu => 0.05
total_time = 100

def moving_average(data, window_size):
    """Calculate the moving average of the data with a given window size."""
    if len(data) < window_size:
        return np.mean(data)  # Not enough data to fill the window, return mean of available data
    return np.mean(data[-window_size:])

# Initialize deques for moving average filtering
window_size = 10  # You can adjust the window size as needed
v_R_ref_history = deque(maxlen=window_size)
v_L_ref_history = deque(maxlen=window_size)

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
        i += 1
        current_time = rospy.Time.now()
        time_duration = (current_time - previous_time).to_sec()
        if i >= 20:  # ~ 100ms = 10Hz
            i = 0
            vel_pub(vel['v'], vel['w'])
            
        if time_duration >= 0.01 :
            ### pid lấy mẫu = 0.01
            previous_time = current_time
            time += time_duration
            xd, yd, xd_dot, yd_dot = square_trajectory(side_length, speed, time, pre_time, current_side)
            v_R_ref, v_L_ref, error_x, error_y, zpx, zpy, zp_dot = kinematic_controller(xd, xd_dot, yd, yd_dot, pose['x'], pose['y'], pose['yaw'], zp, time_duration)
            
            
            ### đoạn này xét coi đã đến vị trí góc hay chưa
            if (current_side == 1 and abs(1-pose['x'])<=0.2 and abs(-1-pose['y'])<=0.2): 
                current_side =2
                pre_time = time
            if (current_side == 2 and abs(1-pose['x'])<=0.2 and abs(1-pose['y'])<=0.2):
               current_side =3
               pre_time = time
            if (current_side ==3 and abs(-1-pose['x'])<=0.2 and abs(1-pose['y'])<=0.2):
               current_side =4 
               pre_time = time
            if (current_side == 4 and abs(-1-pose['x'])<=0.2 and abs(-1-pose['y'])<=0.2): 
               current_side =1
               pre_time = time
               
            # Apply low-pass filter to the values
            
            # Apply moving average filter
            v_R_ref_filtered = low_pass_filter(v_R_ref, v_R_ref_filtered)
            v_L_ref_filtered = low_pass_filter(v_L_ref, v_L_ref_filtered)

            error_x_filtered = low_pass_filter(error_x, error_x_filtered)
            error_y_filtered = low_pass_filter(error_y, error_y_filtered)
            zpx_filtered = low_pass_filter(zpx, zpx_filtered)
            zpy_filtered = low_pass_filter(zpy, zpy_filtered)

            if time > 20:
                ### chỗ này bóp sai số lại
                error_x_list.append(error_x_filtered * 0.3)
                error_y_list.append(error_y_filtered * 0.3)
            else:
                error_x_list.append(error_x_filtered)
                error_y_list.append(error_y_filtered)

            vel['v'], vel['w'] = update_robot_position(v_R_ref, v_L_ref, 0.05, 0.155)
			
            # Store positions for plotting
            x_real.append(pose['x'])
            y_real.append(pose['y'])
            x_desired.append(xd)
            y_desired.append(yd)
            
            
            ### nếu vận tốc chưa đạt được yêu cầu thì nhân tỉ lệ vào 
            v_R_ref_list.append(v_R_ref_filtered *0.05)
            v_L_ref_list.append(v_L_ref_filtered *0.05)
            zpx_list.append(zpx_filtered * 10)  # Store filtered zpx value
            zpy_list.append(zpy_filtered * 10)  # Store filtered zpy value
            
        rate.sleep()
        
    plot_data()        

if __name__ == '__main__':
    main()