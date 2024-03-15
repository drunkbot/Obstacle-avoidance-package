#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist,Point
from std_msgs.msg import Float64
import threading
import math

class bluerov_to_twist:
    def __init__(self):
        rospy.init_node('bluerov_to_twist')
        self.data_lock = threading.RLock()
        self.roll1 = 0.0
        self.pitch1 = 0.0
        self.yaw1 = 0.0
        self.thrust1 = 0.0
        self.vertical_thrust1 = 0.0
        self.lateral_thrust1 = 0.0
        self.roll3 = 0.0
        self.pitch3 = 0.0
        self.yaw3 = 0.0
        self.thrust3 = 0.0
        self.vertical_thrust3 = 0.0
        self.lateral_thrust3 = 0.0
        self.roll2 = 0.0
        self.pitch2 = 0.0
        self.yaw2 = 0.0
        self.thrust2 = 0.0
        self.thrust_set_yaw = 0 
        self.vertical_thrust2 = 0.0
        self.lateral_thrust2 = 0.0
        self.x_danger_on = 1
        self.y_danger_on = 1
        self.x_safe_on = 1
        self.y_safe_on = 1
        self.thrust_set_x = 0.0
        self.lateral_thrust_set_y = 0.0

        self.control_pub = rospy.Publisher('twist', Twist, queue_size=1)
        rospy.Subscriber('cmd_vel3', Twist, self.cmd_vel3_callback)
        rospy.Subscriber('cmd_vel2', Twist, self.cmd_vel2_callback)
        rospy.Subscriber('cmd_vel1', Twist, self.cmd_vel1_callback)
        rospy.Subscriber('/bluerov/twist_setpoint_x', Twist, self.twist_setpoint_x_callback)
        rospy.Subscriber('/bluerov/twist_setpoint_y', Twist, self.twist_setpoint_y_callback)


    def cmd_vel2_callback(self, msg):
        with self.data_lock:
            if msg.angular.x == 5:
                rospy.loginfo("x safe")
                self.x_danger_on = 1
                # self.x_safe_on = 3
            if msg.angular.x == 6:
                rospy.loginfo("x elastic")
                self.x_danger_on = 1
                # self.x_safe_on = 1
            if msg.angular.x == 9:
                rospy.loginfo("x danger")
                # self.x_danger_on = 0 #set to 0 for tether like motion
                self.x_danger_on = 1 #always be 1, for obstacle avoidance
            if msg.angular.y == 5:
                rospy.loginfo("y safe")
                # self.y_safe_on = 3
                self.y_danger_on = 1
            if msg.angular.y == 6:
                rospy.loginfo("y elastic")
                # self.y_safe_on = 1
                self.y_danger_on = 1
            if msg.angular.y == 9:
                rospy.loginfo("y danger")
                # self.y_danger_on = 0 #set to 0 for tether like motion
                self.y_safe_on = 1 #always be 1, for obstacle avoidance
                
            # self.thrust2 =  -0.4*msg.linear.x #experiment tank parameter
            # self.lateral_thrust2 = 0.8*msg.linear.y #experiment tank parameter
            self.thrust2 =  -0.5*msg.linear.x #experiment tank parameter
            self.lateral_thrust2 = 0.8*msg.linear.y #experiment tank parameter
            
            self.vertical_thrust2 = 0
            self.pitch2 = 0
            self.roll2 = 0
            self.yaw2 = msg.angular.z

    def cmd_vel1_callback(self, msg):
        with self.data_lock:
            self.thrust1 = msg.linear.x
            self.lateral_thrust1 = msg.linear.y
            self.vertical_thrust1 = msg.linear.z
            self.pitch1 = msg.angular.y
            self.roll1 = msg.angular.x
            self.yaw1 = msg.angular.z

    def cmd_vel3_callback(self, msg):
        with self.data_lock:
            self.thrust3 = msg.linear.x
            self.lateral_thrust3 = msg.linear.y
            self.vertical_thrust3 = msg.linear.z
            self.pitch3 = msg.angular.y
            self.roll3 = msg.angular.x
            self.yaw3 = msg.angular.z

    def twist_setpoint_x_callback(self, msg):
        with self.data_lock:
            self.thrust_set_x = msg.linear.x  
            self.thrust_set_yaw = msg.angular.z

    def twist_setpoint_y_callback(self, msg):
        with self.data_lock:
            self.lateral_thrust_set_y = msg.linear.y


    def run(self):
        rate = rospy.Rate(50) # 50 Hz
        while not rospy.is_shutdown():
            to_twist = Twist()
            # VVT set up
            to_twist.linear.x = self.thrust1 * self.x_safe_on * self.x_danger_on + self.thrust2 + self.thrust_set_x + self.thrust3
            to_twist.linear.y = self.lateral_thrust1 * self.y_safe_on * self.y_danger_on + self.lateral_thrust2 + self.lateral_thrust_set_y+ self.lateral_thrust3
            to_twist.linear.z = self.vertical_thrust1
            to_twist.angular.x = self.pitch1
            to_twist.angular.y = self.roll1
            to_twist.angular.z = self.yaw1 + self.yaw2 +self.thrust_set_yaw 
            
            # #VVT tro sim experiments for slam problem
            # to_twist.linear.x = self.thrust1 + self.thrust2 + self.thrust_set_x
            # to_twist.linear.y = self.lateral_thrust1 + self.lateral_thrust2 + self.lateral_thrust_set_y
            # to_twist.linear.z = self.vertical_thrust1
            # to_twist.angular.x = self.pitch1
            # to_twist.angular.y = self.roll1
            # to_twist.angular.z = self.yaw1 + self.yaw2 +self.thrust_set_yaw 
            
            self.control_pub.publish(to_twist)

            rate.sleep()
    
def main():
    node = bluerov_to_twist()
    node.run()
        
if __name__ == "__main__":
    main()