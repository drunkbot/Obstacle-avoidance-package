#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist,Point, Quaternion
from std_msgs.msg import Float64
from apriltag_ros.msg import AprilTagDetectionRawArray
from apriltag_ros.msg import AprilTagDetectionArray
import math
import tf

class Virtual_tether:
    def __init__(self):
        rospy.init_node('Virtual_tether')

        self.target = Point(x=320, y=240)
        self.detections = []
        self.danger_d= 0
        self.safe_l = 0
        self.prev_time = None
        self.prev_x = None
        self.prev_y = None
        self.current_x = 0
        self.current_y = 0
        self.goal_x = 0
        self.goal_y = 0
        self.current_velocity = Point()
        self.centre_state_x = 0
        self.centre_state_y = 0
        self.vel_x = 0
        self.vel_y = 0
        self.vel_yaw = 0
        # self.k_p = 1.0
        self.k_d = 0.5
        self.q_yaw = Quaternion()
        self.euler_yaw = 0

        self.control_pub = rospy.Publisher('/mallard/cmd_vel2', Twist, queue_size=1)
        self.eyaw_pub = rospy.Publisher('euler_yaw', Float64, queue_size=1)
        self.qyaw_pub = rospy.Publisher('q_yaw', Quaternion, queue_size=1)
        rospy.Subscriber('tag_detections_raw', AprilTagDetectionRawArray, self.detection_callback)
        rospy.Subscriber('tag_detections', AprilTagDetectionArray, self.yaw_callback)

    def yaw_callback(self,msg):
        if msg.detections:
            self.q_yaw = msg.detections[0].pose.pose.pose.orientation
            quaternion_yaw = [self.q_yaw.x, self.q_yaw.y, self.q_yaw.z, self.q_yaw.w]
            self.euler_yaw = tf.transformations.euler_from_quaternion(quaternion_yaw)[2]
            self.vel_yaw = -1 + ((self.euler_yaw - (-3.14)) * (1 - (-1)) / (3.14 - (-3.14)))
        if abs(self.vel_yaw) > 0.5:
            # If vel_yaw is positive, set it to 0.5; if it's negative, set it to -0.5
            self.vel_yaw = 0.5 if self.vel_yaw > 0 else -0.5
                
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
            # self.vel_x = self.target.x - msg.detections[0].centre.x
            # self.vel_y = self.target.y - msg.detections[0].centre.y
        else:
            self.detections = []
            print('out of LoS!')

    def vel_state(self, img_current, img_target, img_vel, l, d):
        # return vel in (-1 1) and  vel_state
        
        if img_target - 0.5*l < img_current < img_target + 0.5*l:
            v_safe = (img_target - img_current )/(img_target + abs(img_vel)) 
            return 5, v_safe 
        elif img_current > 2 * img_target -d:
            v_danger = -0.8
            return 9, v_danger
        elif  img_current <d:
            v_danger = 0.8
            return 9, v_danger
        else:
            v_tether = (img_target - img_current - img_vel)/(img_target + abs(img_vel)) 
            return 6, v_tether

    def run(self):
        rate = rospy.Rate(50) # 50 Hz
        while not rospy.is_shutdown():
            if self.detections:
            
                # for sim
                x_state, v_x = self.vel_state(self.detections.detections[0].centre.x, self.target.x, self.current_velocity.x, self.safe_l, self.danger_d)
                y_state, v_y = self.vel_state(self.detections.detections[0].centre.y, self.target.y, self.current_velocity.y, self.safe_l, self.danger_d)
                cmd_vel_2 = Twist()
                cmd_vel_2.linear.x = 0.3 * v_y
                cmd_vel_2.linear.y = 0.3 * v_x
                cmd_vel_2.linear.z = 0
                cmd_vel_2.angular.x = y_state
                cmd_vel_2.angular.y = x_state
                cmd_vel_2.angular.z = -0.8*self.vel_yaw
                self.control_pub.publish(cmd_vel_2)
                self.qyaw_pub.publish(self.q_yaw)
                self.eyaw_pub.publish(self.vel_yaw)
                # # for real robots
                # x_state, v_x = self.vel_state(self.detections.detections[0].centre.x, self.target.x, self.current_velocity.x, self.safe_l, self.danger_d)
                # y_state, v_y = self.vel_state(self.detections.detections[0].centre.y, self.target.y, self.current_velocity.y, self.safe_l, self.danger_d)
                # cmd_vel_2 = Twist()
                # cmd_vel_2.linear.x = -0.7*v_y
                # cmd_vel_2.linear.y = 0.7*v_x
                # cmd_vel_2.linear.z = 0
                # cmd_vel_2.angular.x = y_state
                # cmd_vel_2.angular.y = x_state
                # cmd_vel_2.angular.z = self.vel_yaw
                # self.control_pub.publish(cmd_vel_2)

                rate.sleep()
            else:
                cmd_vel_3 = Twist()
                cmd_vel_3.linear.x = 0
                cmd_vel_3.linear.y = 0
                cmd_vel_3.linear.z = 0
                cmd_vel_3.angular.x = 6
                cmd_vel_3.angular.y = 6
                cmd_vel_3.angular.z = 0
                self.control_pub.publish(cmd_vel_3)
                rate.sleep()
    
def main():
    node = Virtual_tether()
    node.run()
        
if __name__ == "__main__":
    main()