
#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import re
import numpy as np

def parse_sonar_data(filename):
    with open(filename, 'r') as f:
        content = f.read()

    # Parsing functions similar to our previous implementations
    # ... (for brevity, not showing here, but will be included in the script)

    # Extract beam directions, ranges, etc.
    # ... 

    return beam_directions, ranges

def numpy_to_pointcloud2(points, frame_id="base_link"):
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1)]

    header = rospy.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    return pc2.create_cloud(header, fields, points)

def sonar_data_to_pointcloud(filename):
    beam_directions, ranges = parse_sonar_data(filename)
    points = [r * np.array(direction) for direction, r in zip(beam_directions, ranges)]
    return points

if __name__ == '__main__':
    rospy.init_node('sonar_pointcloud_publisher')

    # Parse the sonar data from the provided file
    points = sonar_data_to_pointcloud("/path/to/your/projectedsonarimage.txt")

    pub = rospy.Publisher('/sonar_pointcloud', PointCloud2, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        pointcloud_msg = numpy_to_pointcloud2(points)
        pub.publish(pointcloud_msg)
        rate.sleep()
