import rospy
from std_msgs.msg import Float64
import math

'''
subcrible to mallard's scan, get the distance to the wall (left xx, right xx)

subcrible to sum_5_left/right, 

if left > 5 and left distance < xx, mallard_state = 1 ,
if right > 5 and right distance < xx, mallard_state = -1 ,

else mallard_state = 0

publish rotate_angular_z_vel * mallard_state for t1 and sleep for t2


'''

class This_way:
    def __init__(self):
        rospy.init_node('this_way')

        self.
