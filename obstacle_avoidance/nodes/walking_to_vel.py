#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class Walking_to_vel:
    def __init__(self):
        rospy.init_node('walking_to_vel')

        self.linear_x = 0
        self.distance_to_obstacle = 1.0
        self.linear_y = 0
        self.linear_y_state = 1
        self.max_linear_x = 0.2  # Maximum linear_x velocity
        self.max_linear_y = 0.4  # Maximum linear_y velocity

        self.state_sub = rospy.Subscriber('state', Float64, self.state_callback)
        self.distance_sub = rospy.Subscriber('min_dis', Float64, self.distance_callback)
        self.pub_vel = rospy.Publisher('/bluerov/cmd_vel3', Twist, queue_size=1)

    def state_callback(self, msg):
        self.linear_y_state = msg.data
    
    def distance_callback(self, msg):
        self.distance_to_obstacle = msg.data
        if self.distance_to_obstacle < 1.0:
            self.linear_x = -(1.0 - self.distance_to_obstacle) * 0.5
            self.linear_y = min((1.0 - self.distance_to_obstacle) * 2, 0.4)
        else:
            self.linear_x = 0
            # Initiate the decrement of self.linear_y over time here,
            # but actual decrement happens in run() to smoothly decrease value

    def run(self):
        rate = rospy.Rate(10)  # 50 Hz
        increment_per_second = 0.1  # Increment velocity by 0.1 units per second
        increment_per_iteration = increment_per_second / 50  # Increment per loop iteration

        while not rospy.is_shutdown():
            if self.distance_to_obstacle < 1.0:
                # Gradually increase linear_x towards the maximum, if within the affected distance
                self.linear_x = min(self.linear_x + increment_per_iteration, self.max_linear_x)
                
                # Assuming linear_y should also increase in a similar manner
                self.linear_y = min(self.linear_y + increment_per_iteration * self.linear_y_state, self.max_linear_y)
            else:
                # If distance to obstacle is 1.0m or more, decrease linear_y gradually
                if self.linear_y > 0:
                    self.linear_y = max(self.linear_y - (0.1 / 50), 0)

            cmd_vel_3 = Twist()
            cmd_vel_3.linear.x = self.linear_x
            cmd_vel_3.linear.y = self.linear_y * self.linear_y_state
            cmd_vel_3.linear.z = 0
            cmd_vel_3.angular.x = 0
            cmd_vel_3.angular.y = 0
            cmd_vel_3.angular.z = 0
            self.pub_vel.publish(cmd_vel_3)

            rate.sleep()

if __name__ == '__main__':
    try:
        walker = Walking_to_vel()
        walker.run()
    except rospy.ROSInterruptException:
        pass
