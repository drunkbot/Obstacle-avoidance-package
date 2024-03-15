#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def publish_velocity(publisher, angular_z):
    cmd = Twist()
    cmd.angular.z = angular_z
    publisher.publish(cmd)
    rospy.loginfo("Published angular.z = %s to /mallard/cmd_vel3", angular_z)

def main():
    rospy.init_node('keyboard_control')
    pub = rospy.Publisher('/mallard/cmd_vel3', Twist, queue_size=1)
    rate = rospy.Rate(10)  # 10 Hz

    print("Press '+' to publish 10, '-' to publish -10 to /mallard/cmd_vel3, 'q' to quit:")

    while not rospy.is_shutdown():
        key_input = raw_input()  # Use input() if you're using Python 3
        if key_input == '+':
            publish_velocity(pub, 5.0)
        elif key_input == '-':
            publish_velocity(pub, -5.0)
        elif key_input == '0':
            publish_velocity(pub, 0.0)
        elif key_input == 'q':
            break
        else:
            print("Invalid input.")
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
