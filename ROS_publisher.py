#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist


class movement:

    def __init__(self):
        rospy.init_node('move_robot_node', anonymous=False)

        self.pub_move = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.move = Twist()

        """defining instances that they are of type twist"""

    def publish_vel(self):
        self.pub_move.publish(self.move)

        """publishing velocities"""

    def move_forward(self):
        self.move.linear.x = 1
        self.move.angular.z = 0.0

        """setting velocties values"""

    def stop(self):
        self.move.linear.x = 0
        self.move.angular.z = 0.0

        """setting values to stop"""

"""
if __name__ == "__main__":

    mov = movement()
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        movement = input('Enter stop if you want it to stop: ')
        if movement == 'stop':
            mov.stop()

        mov.publish_vel()
        rate.sleep()
"""
if __name__ == "__main__":
    mov = movement()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        movement = input('Enter stop if you want it to stop: ')
        if movement == 'stop':
            mov.stop()
        else:
            mov.move_forward()
    mov.publish_vel()
    rate.sleep()
