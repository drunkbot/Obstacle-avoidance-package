#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
from geometry_msgs.msg import Polygon, Point32

class DepthToDistance:
    def __init__(self):
        rospy.init_node('depth_to_distance_roi', anonymous=True)
        self.bridge = CvBridge()
        self.obstacle_pub = rospy.Publisher('/obstacle_info', Polygon, queue_size=10)
        self.vis_pub = rospy.Publisher('/visualized_depth_image', Image, queue_size=10)
        # self.depth_sub = rospy.Subscriber('/depth_cam/depth/image_raw', Image, self.depth_callback)
        self.depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback)

    def depth_callback(self, data):
        try:
            # Convert the depth image to a Numpy array
            depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
            
            # Apply Gaussian filtering to reduce noise
            depth_image = cv2.GaussianBlur(depth_image, (5,5), 0)

            # Manually set minimum and maximum depth range for visualization
            depth_visualization_min = 0.3  # In meters
            depth_visualization_max = 3  # In meters

            # Clip and normalize the depth image to be in the range [0, 255]
            depth_clipped = np.clip(depth_image, depth_visualization_min, depth_visualization_max)
            depth_normalized = ((depth_clipped - depth_visualization_min) / 
                                (depth_visualization_max - depth_visualization_min) * 255).astype(np.uint8)

            # Convert the single-channel, normalized depth image to a 3-channel image
            depth_colored = cv2.cvtColor(depth_normalized, cv2.COLOR_GRAY2BGR)

            roi_vertical_start = 40
            roi_vertical_end = 440
            roi_horizontal_start = 40
            roi_horizontal_end = 640
            roi = depth_image[roi_vertical_start:roi_vertical_end, roi_horizontal_start:roi_horizontal_end]

            # Obstacle detection based on a depth threshold
            obstacle_threshold = 0.8  # in meters

            # _, thresholded = cv2.threshold(roi, obstacle_threshold, 255, cv2.THRESH_BINARY_INV)
            _, thresholded = cv2.threshold(roi, obstacle_threshold, 255, cv2.THRESH_BINARY_INV)

            # Apply morphological opening to remove small noise regions
            kernel = np.ones((5,5),np.uint8)
            thresholded = cv2.morphologyEx(thresholded, cv2.MORPH_OPEN, kernel)

            thresholded_8u = np.uint8(thresholded)

            # Find contours
            output = cv2.findContours(thresholded_8u, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Unpack the output based on OpenCV version
            contours = output[-2]

            # Initialize an empty Polygon message to hold multiple bounding boxes
            all_bounding_boxes = Polygon()

            # Loop through contours and draw bounding rectangles
            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(depth_colored, (x + roi_horizontal_start, y + roi_vertical_start),
                            (x + w + roi_horizontal_start, y + h + roi_vertical_start), (0, 0, 255), 2)
                
                # Add bounding box points to the Polygon
                all_bounding_boxes.points.append(Point32(x=x, y=y))
                all_bounding_boxes.points.append(Point32(x=x, y=y + h))
                all_bounding_boxes.points.append(Point32(x=x + w, y=y + h))
                all_bounding_boxes.points.append(Point32(x=x + w, y=y))

            # Convert the modified depth image back to a ROS Image message
            vis_msg = self.bridge.cv2_to_imgmsg(depth_colored, "bgr8")
            self.vis_pub.publish(vis_msg)

            # Publish all bounding boxes as a single Polygon
            self.obstacle_pub.publish(all_bounding_boxes)

            rospy.loginfo("Obstacle detected with bounding boxes: {}".format(all_bounding_boxes))

        except Exception as e:
            rospy.logerr("Could not convert from '%s' to 'float32'. " % e)

if __name__ == '__main__':
    node = DepthToDistance()
    rospy.spin()
