#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
import math
import tf

class DogWalking:
    def __init__(self):
        rospy.init_node('dog_walking')

        self.min_group_size = 8  # Minimum number of contiguous points to be considered a valid group
        self.subscriber = rospy.Subscriber('scan_sonar', LaserScan, self.scan_callback)
        self.obstacle_distance_threshold = 0.66  # meters, you can adjust this
        self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        self.distance_publisher = rospy.Publisher('min_dis', Float64 , queue_size=10)
        self.average_distance_publisher = rospy.Publisher('average_dis', Float64, queue_size=10)

        # self.movement_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    def scan_callback(self, msg):
        # Filter out 'inf' values and calculate the minimum distance
        valid_distances = [d for d in msg.ranges if d != float('inf')]
        min_distance = min(valid_distances) if valid_distances else float('inf')
        self.distance_publisher.publish(min_distance)
        # Check if the minimum distance is below the threshold
        if min_distance < self.obstacle_distance_threshold:
            rospy.loginfo("Obstacle detected " + str(min_distance) + " meters away. Taking evasive action.")
            self.avoid_obstacle()
        else:
            rospy.loginfo("Path clear. Proceeding normally.")
            self.normal_walking()
        
        cartesian_points = self.convert_to_cartesian(msg.ranges, msg.angle_min, msg.angle_increment)
        if not cartesian_points:
            rospy.loginfo("No valid data points detected for bounding box calculation.")
            self.clear_previous_bounding_box()
        else:
            # Calculate PCA orientation
            angle = self.calculate_pca_orientation(cartesian_points)
            
            # Rotate points
            rotated_points = [self.rotate_point(p, -angle) for p in cartesian_points]

            # Calculate bounding box on rotated points
            bounding_box = self.calculate_bounding_box(rotated_points)

            # Publish bounding box marker (with rotation applied)
            self.publish_bounding_box_marker(bounding_box, angle)
            self.calculate_and_publish_average_distance(valid_distances)

    def calculate_and_publish_average_distance(self, distances):
        if distances:
            average_distance = sum(distances) / len(distances)
            self.average_distance_publisher.publish(average_distance)
            rospy.loginfo("Average distance to obstacles: {:.2f} meters".format(average_distance))
        else:
            rospy.loginfo("No valid distances to calculate average.")

    def convert_to_cartesian(self, ranges, angle_min, angle_increment):
        max_range = 1.2000000476837158  # Define the max range value indicating no effective reading
        cartesian_points = []
        current_angle = angle_min
        current_group = []

        for r in ranges:
            if r < max_range:
                x = r * math.cos(current_angle)
                y = r * math.sin(current_angle)
                current_group.append((x, y))
            else:
                if len(current_group) >= self.min_group_size:
                    cartesian_points.extend(current_group)
                current_group = []
            current_angle += angle_increment

        # Check the last group
        if len(current_group) >= self.min_group_size:
            cartesian_points.extend(current_group)

        return cartesian_points

    def calculate_pca_orientation(self, points):
        import numpy as np
        if not points:
            return 0  # Default orientation if no points

        data = np.array(points)
        mean = np.mean(data, axis=0)
        data_centered = data - mean
        covariance_matrix = np.cov(data_centered.T)
        eigenvalues, eigenvectors = np.linalg.eig(covariance_matrix)

        # Eigenvector corresponding to the largest eigenvalue
        principal_eigenvector = eigenvectors[:, np.argmax(eigenvalues)]
        angle = math.atan2(principal_eigenvector[1], principal_eigenvector[0])

        return angle

    def rotate_point(self, point, angle):
        x_rotated = point[0] * math.cos(angle) - point[1] * math.sin(angle)
        y_rotated = point[0] * math.sin(angle) + point[1] * math.cos(angle)
        return x_rotated, y_rotated

    def clear_previous_bounding_box(self):
        clear_marker = Marker()
        clear_marker.header.frame_id = "sonar_frame"  # Set to your sonar sensor's frame
        clear_marker.action = Marker.DELETEALL  # This deletes all markers in the topic
        self.marker_publisher.publish(clear_marker)


    def publish_bounding_box_marker(self, bounding_box, angle): 
        marker = Marker()
        marker.header.frame_id = "obstacle_frame"  # Or the appropriate frame ID
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Set the marker scale and color
        marker.scale.x = 0.05  # Line width
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Alpha

        # Define the corners of the bounding box
        corners = [
            (bounding_box['min_x'], bounding_box['min_y']),
            (bounding_box['min_x'], bounding_box['max_y']),
            (bounding_box['max_x'], bounding_box['max_y']),
            (bounding_box['max_x'], bounding_box['min_y']),
            (bounding_box['min_x'], bounding_box['min_y'])  # Close the loop
        ]
        rotated_corners = [self.rotate_point(c, angle) for c in corners]

        # Create points for each corner
        for corner in rotated_corners:
            p = Point()
            p.x = corner[0]
            p.y = corner[1]
            p.z = 0  # Assuming a 2D plane
            marker.points.append(p)

        # Publish the marker
        self.marker_publisher.publish(marker)


    def calculate_bounding_box(self, points):
        min_x = min(points, key=lambda p: p[0])[0]
        max_x = max(points, key=lambda p: p[0])[0]
        min_y = min(points, key=lambda p: p[1])[1]
        max_y = max(points, key=lambda p: p[1])[1]
        return {'min_x': min_x, 'max_x': max_x, 'min_y': min_y, 'max_y': max_y}

    def calculate_rectangle_area(self, bounding_box):
        width = bounding_box['max_x'] - bounding_box['min_x']
        height = bounding_box['max_y'] - bounding_box['min_y']
        return width * height

    def avoid_obstacle(self):
        rospy.loginfo("Taking evasive action!!!")
        # Implement obstacle avoidance behavior here
        # For example: stop or turn
        # twist = Twist()
        # twist.linear.x = 0  # stop
        # twist.angular.z = 1.0  # turn
        # self.movement_publisher.publish(twist)

    def normal_walking(self):
        rospy.loginfo("Proceeding normally!!!")
        # Implement normal walking behavior here
        # twist = Twist()
        # twist.linear.x = 1.0  # move forward
        # self.movement_publisher.publish(twist)

def publish_transform():
    broadcaster = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        # Broadcasting the transformation
        broadcaster.sendTransform(
            (0, 0, 0),  # No translation
            tf.transformations.quaternion_from_euler(0, 0, -math.pi/2),  # 90 degrees rotation around Z
            rospy.Time.now(),
            "obstacle_frame",  # Child frame (new frame)
            "base_link_b"  # Parent frame
        )
        rospy.sleep(0.1)

if __name__ == '__main__':
    dog_walking_robot = DogWalking()
    publish_transform()
    rospy.spin()
