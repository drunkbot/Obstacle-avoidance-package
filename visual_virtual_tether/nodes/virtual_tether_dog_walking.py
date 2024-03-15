#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist,Point, Quaternion
from std_msgs.msg import Float64
from apriltag_ros.msg import AprilTagDetectionRawArray
from apriltag_ros.msg import AprilTagDetectionArray
import math
from std_msgs.msg import Int32
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
        self.prev_time = None  # Initialize prev_time as None

        self.sum_5_left = 0
        self.sum_5_right = 0
        self.sum_5_forward = 0
        self.sum_5_backward = 0

        # Initialize dictionaries to keep track of the timestamps for each area
        self.last_time_in_area = {'left': 0, 'right': 0, 'forward': 0, 'backward': 0}
        self.time_accumulator = {'left': 0, 'right': 0, 'forward': 0, 'backward': 0}
        self.last_increment_time = {'left': None, 'right': None, 'forward': None, 'backward': None}
        self.accumulated_time = {'left': 0, 'right': 0, 'forward': 0, 'backward': 0}
        current_time = rospy.get_time()
        self.last_decrement_time = {'left': current_time, 'right': current_time, 
                                    'forward': current_time, 'backward': current_time}


        self.control_pub = rospy.Publisher('cmd_vel2', Twist, queue_size=1)
        self.eyaw_pub = rospy.Publisher('euler_yaw', Float64, queue_size=1)
        self.qyaw_pub = rospy.Publisher('q_yaw', Quaternion, queue_size=1)
        self.pub_sum_5_left = rospy.Publisher('sum_5_left', Int32, queue_size=10)
        self.pub_sum_5_right = rospy.Publisher('sum_5_right', Int32, queue_size=10)
        self.pub_sum_5_forward = rospy.Publisher('sum_5_forward', Int32, queue_size=10)
        self.pub_sum_5_backward = rospy.Publisher('sum_5_backward', Int32, queue_size=10)
        rospy.Subscriber('tag_detections_raw', AprilTagDetectionRawArray, self.detection_callback)
        rospy.Subscriber('tag_detections', AprilTagDetectionArray, self.yaw_callback)

    def yaw_callback(self,msg):
        if msg.detections:
            self.q_yaw = msg.detections[0].pose.pose.pose.orientation
            quaternion_yaw = [self.q_yaw.x, self.q_yaw.y, self.q_yaw.z, self.q_yaw.w]
            self.euler_yaw = tf.transformations.euler_from_quaternion(quaternion_yaw)[2]
            self.vel_yaw = 0.2*(-1 + ((self.euler_yaw - (-3.14*0.5)) * (1 - (-1)) / (3.14*0.5 - (-3.14*0.5))))
        if abs(self.vel_yaw) > 0.5:
            # If vel_yaw is positive, set it to 0.5; if it's negative, set it to -0.5
            self.vel_yaw = 0.5 if self.vel_yaw > 0 else -0.5
                
    def detection_callback(self, msg):
        current_time = rospy.get_time()

        if msg.detections :
            self.detections = msg
            self.update_area_times(msg, current_time)

            self.publish_area_sums()            

            if self.prev_x is not None and self.prev_y is not None and self.prev_time is not None:
                dt = current_time - self.prev_time
                if dt > 1e-6:
                    self.current_velocity.x = (msg.detections[0].centre.x - self.prev_x) / dt
                    self.current_velocity.y = (msg.detections[0].centre.y - self.prev_y) / dt
                else:
                # If dt is zero, we can't compute the velocity, so we can either skip updating
                # the velocity or handle it in another suitable manner
                    rospy.logwarn("Delta time (dt) is zero, skipping velocity calculation")
            else:
                rospy.loginfo("Previous time not set, cannot calculate velocity")
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

    def update_area_times(self, msg, current_time):
        # Check the x and y coordinates to determine area
        x_area = self.get_area(msg.detections[0].centre.x, 'x')
        y_area = self.get_area(msg.detections[0].centre.y, 'y')

        # Update times for each area
        for area in ['left', 'right', 'forward', 'backward']:
            if x_area == area or y_area == area:
                if current_time - self.last_time_in_area[area] >= 1:
                    self.time_accumulator[area] += 1
                    self.time_accumulator[area] = min(4, self.time_accumulator[area])

                    self.last_time_in_area[area] = current_time
                    pass
            else:
                # Check if 0.5 seconds have passed since the last decrement
                if current_time - self.last_decrement_time[area] > 0.5:
                    self.time_accumulator[area] = max(0, self.time_accumulator[area] - 1)
                    # Update the last decrement time as a float
                    self.last_decrement_time[area] = current_time

    def get_area(self, value, axis):
        if axis == 'x':
            if 0 <= value < 200:
                return 'left'
            elif 440 <= value <= 640:
                return 'right'
        elif axis == 'y':
            if 0 <= value < 140:
                return 'forward'
            elif 340 <= value <= 480:
                return 'backward'
        return None
    
    def publish_area_sums(self):
        # Publish the area sums
        self.pub_sum_5_left.publish(self.time_accumulator['left'])
        self.pub_sum_5_right.publish(self.time_accumulator['right'])
        self.pub_sum_5_forward.publish(self.time_accumulator['forward'])
        self.pub_sum_5_backward.publish(self.time_accumulator['backward'])


    def vel_state(self, img_current, img_target, img_vel, l, d):
        # return vel in (-1 1) and  vel_state
        
        if img_target - 0.5*l < img_current < img_target + 0.5*l:
            v_safe = (img_target - img_current )/(img_target + abs(img_vel)) 
            return 5, v_safe 
        elif img_current > 2 * img_target -d:
            v_danger = -1.3
            return 9, v_danger
        elif  img_current <d:
            v_danger = 1.3
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

                # cmd_vel_2.linear.x = 0.7*v_y #experiment tank parameter
                # cmd_vel_2.linear.y = 0.7*v_x #experiment tank parameter
                cmd_vel_2.linear.x = v_y #sim parameter
                cmd_vel_2.linear.y = v_x #sim parameter

                cmd_vel_2.linear.z = 0
                cmd_vel_2.angular.x = y_state
                cmd_vel_2.angular.y = x_state

                # cmd_vel_2.angular.z = -0.8*self.vel_yaw #experiment tank parameter
                cmd_vel_2.angular.z = -0.07*self.vel_yaw #sim parameter

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
                # cmd_vel_2.angular.z = -self.vel_yaw
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