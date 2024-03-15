#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist,Point, Quaternion
from std_msgs.msg import Float64
from apriltag_ros.msg import AprilTagDetectionRawArray
from apriltag_ros.msg import AprilTagDetectionArray
from obstacle_detector.msg import Obstacles
import math
import tf
from visualization_msgs.msg import Marker


class Obstacle_vel:
    def __init__(self):
        rospy.init_node('Obstacle_vel')

        self.force_direction = 1  # Use 1 for clockwise, -1 for counterclockwise
        self.robot_position = [0, 0, 0]  # Assuming robot is at the origin
        self.distances = []
        self.centers = []
        self.linear_x = 0
        self.linear_y = 0
        self.control_pub = rospy.Publisher('cmd_vel3', Twist, queue_size=1)
        rospy.Subscriber('visualization_marker', Marker, self.Obstacles_callback)


    def Obstacles_callback(self, msg):
        # msg = msg.points
        if msg:
            self.centers = [(p.x, p.y) for p in msg.points[:3]]  # Assuming the first four points define the rectangle
            self.calculate_repulsive_force()


    def get_edge_directions(self, corners):
        # Corners are assumed to be in clockwise order
        edges = []
        num_corners = len(corners)
        for i in range(num_corners):
            next_index = (i + 1) % num_corners  # To loop back to the first point after the last one
            edge = (corners[next_index][0] - corners[i][0], corners[next_index][1] - corners[i][1])
            edges.append(edge)
        return edges



    def calculate_repulsive_force(self):
        force_x, force_y = 0, 0
        for edge in self.get_edge_directions(self.centers):
            # Normalizing edge direction
            edge_length = math.sqrt(edge[0]**2 + edge[1]**2)
            if edge_length == 0:
                continue

            # Determine edge direction (clockwise or counterclockwise)
            direction_x, direction_y = -edge[1] * self.force_direction, edge[0] * self.force_direction

            # Calculate force magnitude based on distance to (0,0)
            midpoint = ((edge[0] / 2), (edge[1] / 2))  # Midpoint of the edge
            distance = math.sqrt(midpoint[0]**2 + midpoint[1]**2)
            if distance != 0:
                force_magnitude = 1 / distance
                rospy.loginfo("distances".format(distance))
                # Apply force magnitude to normalized direction
                force_x += direction_x * force_magnitude
                force_y += direction_y * force_magnitude

        self.linear_x, self.linear_y = force_x, force_y
        rospy.loginfo("Force X: {}, Force Y: {}".format(force_x, force_y))
            

    def run(self):
        rate = rospy.Rate(50) # 50 Hz
        while not rospy.is_shutdown():
            cmd_vel_3 = Twist()
            cmd_vel_3.linear.x = self.linear_y
            cmd_vel_3.linear.y =self.linear_x
            cmd_vel_3.linear.z = 0
            cmd_vel_3.angular.x = 0
            cmd_vel_3.angular.y = 0
            cmd_vel_3.angular.z = 0
            self.control_pub.publish(cmd_vel_3)

            rate.sleep()

    
def main():
    node = Obstacle_vel()
    node.run()
        
if __name__ == "__main__":
    main()