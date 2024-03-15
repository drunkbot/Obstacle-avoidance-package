#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist,Point
from std_msgs.msg import Float64
import numpy as np
from apriltag_ros.msg import AprilTagDetectionRawArray
import math

class KF_prediction:
    def __init__(self):
        rospy.init_node('KF_prediction')


        self.detections = []
        self.dt = 0
        self.prev_time = None
        self.prev_x = None
        self.prev_y = None
        self.current_x = 0
        self.current_y = 0
        self.current_velocity = Point()
        A = np.array([[1, 0, self.dt, 0],
                [0, 1, 0, self.dt],
                [0, 0, 1, 0],
                [0, 0, 0, 1]])

        # self.k_p = 1.0
        # self.k_d = 1.0
        self.estimated = AprilTagDetectionRawArray()
        self.control_pub = rospy.Publisher('tag_prediction', AprilTagDetectionRawArray, queue_size=1)
        rospy.Subscriber('/bluerov/tag_detections_raw', AprilTagDetectionRawArray, self.detection_callback)

    def detection_callback(self, msg):
        current_time = rospy.Time.now().to_sec()

        if msg.detections :
            self.detections = msg
            if self.prev_x is not None and self.prev_y is not None and self.prev_time is not None:
                dt = current_time - self.prev_time
                self.current_velocity.x = (msg.detections[0].centre.x - self.prev_x) / dt
                self.current_velocity.y = (msg.detections[0].centre.y - self.prev_y) / dt

            self.prev_x = msg.detections[0].centre.x
            self.prev_y = msg.detections[0].centre.y
            self.prev_time = current_time

            self.danger_d = 0.5 * (math.sqrt((self.detections.detections[0].corners.top_right.x-self.detections.detections[0].corners.bottom_left.x)**2 \
                                            + (self.detections.detections[0].corners.top_right.y-self.detections.detections[0].corners.bottom_left.y)**2) \
                                                +math.sqrt((self.detections.detections[0].corners.top_left.x-self.detections.detections[0].corners.bottom_right.x)**2 \
                                                        + (self.detections.detections[0].corners.top_left.y-self.detections.detections[0].corners.bottom_right.y)**2))
            self.safe_l = 0.25 * (math.sqrt((self.detections.detections[0].corners.top_right.x-self.detections.detections[0].corners.top_left.x)**2 \
                                            + (self.detections.detections[0].corners.top_right.y-self.detections.detections[0].corners.top_left.y)**2) \
                                            + math.sqrt((self.detections.detections[0].corners.top_right.x-self.detections.detections[0].corners.bottom_right.x)**2 \
                                            + (self.detections.detections[0].corners.top_right.y-self.detections.detections[0].corners.bottom_right.y)**2) \
                                            + math.sqrt((self.detections.detections[0].corners.bottom_right.x-self.detections.detections[0].corners.bottom_left.x)**2 \
                                            + (self.detections.detections[0].corners.bottom_right.y-self.detections.detections[0].corners.bottom_left.y)**2) \
                                            + math.sqrt((self.detections.detections[0].corners.bottom_left.x-self.detections.detections[0].corners.top_left.x)**2 \
                                            + (self.detections.detections[0].corners.bottom_left.y-self.detections.detections[0].corners.top_left.y)**2))
        else:
            print('out of LoS!')
    def run(self):
        rate = rospy.Rate(50) # 10 Hz

        while not rospy.is_shutdown():
            

            cmd_vel_2 = Twist()
            cmd_vel_2.linear.x = self.danger_d
            cmd_vel_2.linear.y = self.safe_l
            cmd_vel_2.linear.z = 0
            cmd_vel_2.angular.x = self.current_velocity.x 
            cmd_vel_2.angular.y = self.current_velocity.y
            cmd_vel_2.angular.z = 0
            self.control_pub.publish(cmd_vel_2)

            rate.sleep()
    
def main():
    node = KF_prediction()
    node.run()
        
if __name__ == "__main__":
    main()