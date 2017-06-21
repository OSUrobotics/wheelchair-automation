#!/usr/bin/env python
import roslib
roslib.load_manifest('wheelchair_navigation')
import rospy, message_filters
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Joy
import numpy as np
import std_msgs.msg
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal


class nav_clear:
    def __init__(self):
        self.joy_sub = rospy.Subscriber('/joy',Joy,self.joy_callback)
        # self.move_goal = rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, self.newGoalHandler)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Wating for move_base server.....")
        self.client.wait_for_server()

    def joy_callback(self,joy):
        if (joy.buttons[2] == 1):
            print "joy_callback"
            self.client.cancel_goal()
            self.client.cancel_all_goals()
            rospy.loginfo("Goal cancelled")
        else:
            pass

if __name__ == '__main__':
	rospy.init_node('nav_joysick',log_level=rospy.DEBUG)
	nav_clear = nav_clear()
	rospy.spin()
