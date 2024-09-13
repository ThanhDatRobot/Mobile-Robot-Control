#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def main():
    rospy.init_node('stop_robot_node', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    rate = rospy.Rate(10)  # 10 Hz
    
    while not rospy.is_shutdown():
        vel_msg = Twist()
        vel_msg.linear.x = 0.0  # linear velocity (m/s)
        vel_msg.angular.z = 0.0  # angular velocity (rad/s)
        print("The robot is stopping!!!")
        pub.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


