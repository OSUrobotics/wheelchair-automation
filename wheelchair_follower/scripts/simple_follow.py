#!/usr/bin/env python
import roslib
roslib.load_manifest('wheelchair_follower')
import rospy, message_filters
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Joy
import numpy as np
import std_msgs.msg
from visualization_msgs.msg import Marker, MarkerArray

class follower:
    def __init__(self):
        self.past_marker = Marker()
        self.past_marker.ns = "NONE"
        self.mem_sub = rospy.Subscriber('/visualization_marker',Marker,self.mem_callback)
        self.joy_sub = rospy.Subscriber('/joy',Joy,self.joy_callback)
        self.joy_bool = False
        self.move_sub = rospy.Subscriber('/visualization_marker',Marker,self.move_callback)

        self.follower_twist = rospy.Publisher('/cmd_vel', Twist,queue_size=3)
        self.command = Twist()


    def move_callback(self,marker):
        if (self.joy_bool == True):
            if (self.past_marker.ns == "NONE"):
                return 0
            else:
                x_vect = (marker.pose.position.x - self.past_marker.pose.position.x)
                y_vect = (marker.pose.position.y - self.past_marker.pose.position.y)
                self.command.linear.x = x_vect
                self.command.linear.y = y_vect
                self.follower_twist.publish(self.command)

        else:
            return 0

    def joy_callback(self,joy):
        if (joy.buttons[3] == 1):
            self.joy_bool = True
        else:
            self.joy_bool = False

    def mem_callback(self,marker):
        if (marker.ns == "PEOPLE") == True:
            self.past_marker = marker
        else:
            return 0

if __name__ == '__main__':
	rospy.init_node('wheelchair_follower_controller',log_level=rospy.DEBUG)
	follower = follower()
	rospy.spin()
