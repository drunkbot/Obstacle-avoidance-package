#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist,Point
from std_msgs.msg import Float64,Int32
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
        self.left_k2 = 1
        self.right_k2 = 1
        self.forward_k2 =1 
        self.backward_k2 =1 
        self.left_k1 = 1
        self.right_k1 = 1
        self.forward_k1 =1 
        self.backward_k1 =1 
        self.linearx3 = 0
        self.lineary3 = 0
        self.angular3 = 0


        self.control_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('cmd_vel2', Twist, self.cmd_vel2_callback)
        rospy.Subscriber('cmd_vel1', Twist, self.cmd_vel1_callback)
        rospy.Subscriber('cmd_vel3', Twist, self.cmd_vel3_callback)
        rospy.Subscriber('sum_5_left', Int32, self.left_callback)
        rospy.Subscriber('sum_5_right', Int32, self.right_callback)
        rospy.Subscriber('sum_5_forward', Int32, self.forward_callback)
        rospy.Subscriber('sum_5_backward', Int32, self.backward_callback)

    def left_callback(self, msg):
        value = msg.data
        if value >=4:
            self.left_k1 = 1
            # self.left_k2 = 1.3
        else:
            self.left_k1 = 1
            self.left_k2 = 1

    def right_callback(self, msg):
        value = msg.data
        if value >=4:
            self.right_k1 = 1
            # self.right_k2 = 1.3
        else:
            self.right_k1 = 1
            self.right_k2 = 1

    def forward_callback(self, msg):
        value = msg.data
        if value >=4:
            self.forward_k1 = 0
            self.forward_k2 = 1.1
        else:
            self.forward_k1 = 1
            self.forward_k2 = 1
            
    def backward_callback(self, msg):
        value = msg.data
        if value >=4:
            self.backward_k1 = 0
            self.backward_k2 = 1.1
        else:
            self.backward_k1 = 1
            self.backward_k2 = 1
    def cmd_vel2_callback(self, msg):
        with self.data_lock:
            if msg.angular.x == 5:
                rospy.loginfo("x safe")
                self.thrust2 = 2.5*msg.linear.x
                self.x_danger_on = 1
                self.x_safe_on = 1
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
                self.y_safe_on = 1

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
            self.yaw2 = -0.5*msg.angular.z

    def cmd_vel1_callback(self, msg):
        with self.data_lock:
            self.thrust1 = 1*msg.linear.x
            self.lateral_thrust1 = 1*msg.linear.y
            self.vertical_thrust1 = 0
            self.pitch1 = 0
            self.roll1 = 0
            self.yaw1 = msg.angular.z

    def cmd_vel3_callback(self, msg):
        self.linearx3 = msg.linear.x
        self.lineary3 = msg.linear.y
        self.angular3 = msg.angular.z

    def run(self):
        rate = rospy.Rate(50) # 50 Hz
        while not rospy.is_shutdown():
            to_twist = Twist()
            
            #experiemts
            # to_twist.linear.x = 0.5*(self.thrust1 * self.x_safe_on * self.x_danger_on + self.thrust2)
            # to_twist.linear.y = 0.5*(self.lateral_thrust1 * self.y_safe_on * self.y_danger_on + self.lateral_thrust2)
            
            #sim
            to_twist.linear.x = (self.forward_k1*self.backward_k1*self.thrust1 * self.x_safe_on * self.x_danger_on 
                                 + self.backward_k2*self.forward_k2*self.thrust2) +self.linearx3
            to_twist.linear.y = 0.5*(self.left_k1*self.right_k1*self.lateral_thrust1 * self.y_safe_on * self.y_danger_on
                                      + self.left_k2*self.right_k2*self.lateral_thrust2) +self.lineary3

            # #sim for vvt slam problem 0813
            # to_twist.linear.x = (self.thrust1 + 0*self.thrust2)
            # to_twist.linear.y = 0.8*(self.lateral_thrust1 + 0*self.lateral_thrust2)


            to_twist.linear.z = self.vertical_thrust1
            to_twist.angular.x = self.pitch1
            to_twist.angular.y = self.roll1
            to_twist.angular.z = self.yaw1 + self.angular3
            self.control_pub.publish(to_twist)

            rate.sleep()
    
def main():
    node = mallard_to_twist()
    node.run()
        
if __name__ == "__main__":
    main()