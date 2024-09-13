#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import random
from itertools import count
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure
from matplotlib.animation import FuncAnimation


class Info:
    x1 = 0
    x2 = 0
    z1 = 0
    z2 = 0
    legend_1 = False
    legend_2 = False
    time = 100
    k = time / 1000
    index = 0

plt.style.use('fivethirtyeight')
t = []

x_vals = []
z_vals = []

x_ref = []
z_ref = []


def animate(i):
    plt.cla()
    
    plt.suptitle("Đồ thị vận tốc")

    t.append(Info.index)

    Info.index += Info.k

    x_vals.append(Info.x1)
    x_ref.append(Info.x2)
    z_vals.append(Info.z1)
    z_ref.append(Info.z2)
    
    plt.subplot(2, 1, 1)
    plt.plot(t, x_vals, linewidth=1, color='green', label="v_val")
    plt.plot(t, x_ref, linewidth=1, color='blue', label="v_ref")
    plt.xlabel('T(s)')
    plt.ylabel('v(m/s)')
    if Info.legend_1 == False: 
        plt.legend(loc="upper right")
        Info.legend_1 = True

    #===============================================================

    plt.subplot(2, 1, 2)
    plt.plot(t, z_vals, linewidth=1, color='green', label="w_val")
    plt.plot(t, z_ref, linewidth=1, color='blue', label="w_ref")
    plt.xlabel('T(s)')
    plt.ylabel('w(rad/s)')
    # if Info.legend_2 == False: 
    #     plt.legend()
    #     Info.legend_2 = True
    plt.legend(loc="upper right")

def subvel_callback(msg: Twist):
    rospy.loginfo("(" + str(msg.linear.x) + ", " + str(msg.angular.z) + ")")
    Info.x2 = float(msg.linear.x)
    Info.z2 = float(msg.angular.z)

def velsub_callback(vel_data: Twist):
    rospy.loginfo("(" + str(vel_data.linear.x) + ", " + str(vel_data.angular.z) + ")")
    Info.x1 = float(vel_data.linear.x)
    Info.z1 = float(vel_data.angular.z)

if __name__ == '__main__':
    rospy.init_node("velocity_subscriber")

    a = rospy.Subscriber("cmd_vel", Twist, callback=subvel_callback)

    b = rospy.Subscriber("vel_pub", Twist, callback=velsub_callback)
    
    ani = FuncAnimation(plt.gcf(), animate, interval=Info.time)

    plt.gcf().set_size_inches(12, 8)
    plt.tight_layout()
    plt.show()

    rospy.spin() 