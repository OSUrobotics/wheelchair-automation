#!/usr/bin/env python

import rospy
from actionlib_msgs.msg import GoalID
from sensor_msgs.msg import Joy

class NavOff(object):
    def __init__(self):
	rospy.init_node('nav_off')
	rospy.Subscriber("joy", Joy, self.joy_callback)


        self.publisher = rospy.Publisher('move_base/cancel', GoalID, queue_size=10)
	self.nav_off = False
        rospy.spin()

    def joy_callback(self, joy_msg):
	if joy_msg.buttons[0] == 1 and self.nav_off == False:
	    print("TURNED OFF NAV")
	    emptyGoal = GoalID()
	    self.publisher.publish(emptyGoal)
	    self.nav_off = True
	elif joy_msg.buttons[0] == 0:
	    self.nav_off = False
if __name__== '__main__':
    NavOff()

