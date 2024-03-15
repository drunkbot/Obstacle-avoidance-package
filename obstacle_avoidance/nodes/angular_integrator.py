#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8
from std_msgs.msg import Float64
import math

class AngularIntegrator:
    def __init__(self):
        rospy.init_node('angular_integrator')

        self.state_pub = rospy.Publisher('state', Float64, queue_size=1)
        # Ensure the message type matches your detection messages
        self.detections_sub = rospy.Subscriber('/bluerov/euler_yaw', Float64, self.detections_callback)
        self.state = 0
        self.angle = 0  
        self.change_threshold_degrees = 0.35

    def detections_callback(self, msg):
        if msg:
            self.angle = msg.data
            if abs(self.angle) < self.change_threshold_degrees:
                if self.angle > 0: 
                    self.state = -1 
                elif self.angle < 0:
                    self.state = 1
                else: 
                    rospy.loginfo("angle error")
            else: 
                rospy.loginfo("forward!")
            self.state_pub.publish(self.state)

if __name__ == '__main__':
    try:
        angular_integrator = AngularIntegrator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass