#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist,Point
from std_msgs.msg import Float64
import threading
import math

class mallard_to_twist:
    def __init__(self):
        rospy.init_node('mallard_to_twist')
        self.data_lock = threading.RLock()
        self.roll1 = 0.0
        self.pitch1 = 0.0
        self.yaw1 = 0.0
        self.thrust1 = 0.0
        self.vertical_thrust1 = 0.0
        self.lateral_thrust1 = 0.0
        self.roll2 = 0.0
        self.pitch2 = 0.0
        self.yaw2 = 0.0
        self.thrust2 = 0.0
        self.vertical_thrust2 = 0.0
        self.lateral_thrust2 = 0.0
        self.x_danger_on = 1
        self.y_danger_on = 1
        self.x_safe_on = 1
        self.y_safe_on = 1

        self.control_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('cmd_vel2', Twist, self.cmd_vel2_callback)
        rospy.Subscriber('cmd_vel1', Twist, self.cmd_vel1_callback)


    def cmd_vel2_callback(self, msg):
        with self.data_lock:
            if msg.angular.x == 5:
                rospy.loginfo("x safe")
                self.thrust2 = 2.5*msg.linear.x
                self.x_danger_on = 1
                self.x_safe_on = 3
            if msg.angular.x == 6:
                rospy.loginfo("x elastic")
                self.thrust2 = 2.5*msg.linear.x
                self.x_danger_on = 1
                self.x_safe_on = 1
            if msg.angular.x == 9:
                rospy.loginfo("x danger")
                self.x_danger_on = 0
                self.thrust2 = 2.5*msg.linear.x 
                self.x_safe_on = 1  

            if msg.angular.y == 5:
                rospy.loginfo("y safe")
                self.lateral_thrust2 = 3*msg.linear.y
                self.y_danger_on = 1
                self.y_safe_on = 3

            if msg.angular.y == 6:
                rospy.loginfo("y elastic")
                self.lateral_thrust2 = 3*msg.linear.y
                self.y_danger_on = 1
                self.y_safe_on = 1

            if msg.angular.y == 9:
                rospy.loginfo("y danger")
                self.y_danger_on = 0
                self.y_safe_on = 1
                self.thrust2 = 3*msg.linear.y   
            # self.thrust2 = -0.4*msg.linear.x
            # self.lateral_thrust2 = 0.8*msg.linear.y
            self.vertical_thrust2 = 0
            self.pitch2 = 0
            self.roll2 = 0
            self.yaw2 = 0

    def cmd_vel1_callback(self, msg):
        with self.data_lock:
            self.thrust1 = msg.linear.x
            self.lateral_thrust1 = msg.linear.y
            self.vertical_thrust1 = 0
            self.pitch1 = 0
            self.roll1 = 0
            self.yaw1 = msg.angular.z

    def run(self):
        rate = rospy.Rate(50) # 50 Hz
        while not rospy.is_shutdown():
            to_twist = Twist()
            to_twist.linear.x = 0.5*(self.thrust1 )
            to_twist.linear.y = 0.5*(self.lateral_thrust1)
            to_twist.linear.z = self.vertical_thrust1
            to_twist.angular.x = self.pitch1
            to_twist.angular.y = self.roll1
            to_twist.angular.z = self.yaw1
            self.control_pub.publish(to_twist)

            rate.sleep()
    
def main():
    node = mallard_to_twist()
    node.run()
        
if __name__ == "__main__":
    main()