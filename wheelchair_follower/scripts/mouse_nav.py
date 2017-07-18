#!/usr/bin/env python
import roslib
roslib.load_manifest('wheelchair_follower')
import rospy, message_filters
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Joy
import numpy as np
import std_msgs.msg
from visualization_msgs.msg import Marker, MarkerArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

class follower:
	def __init__(self):
		self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		self.position_marker = Marker()
		self.position_marker.ns = "NONE"

		self.sub1 = message_filters.Subscriber('/visualization_marker', Marker)
		self.sub2 = message_filters.Subscriber('/joy', Joy)
		self.ts = message_filters.ApproximateTimeSynchronizer([self.sub1,self.sub2], 10, 4)
		self.ts.registerCallback(self.set_position_callback)

		# self.set_position = None
		self.joy_sub = rospy.Subscriber('/joy',Joy,self.joy_callback)
		# self.mem_sub = rospy.Subscriber('/visualization_marker',Marker,self.mem_callback)
		self.joy_bool = False
		self.move_sub = rospy.Subscriber('/visualization_marker',Marker,self.move_callback)
		self.ref_viz_pub = rospy.Publisher('ref_marker', Marker, queue_size=10)

		self.follower_twist = rospy.Publisher('/cmd_vel', Twist,queue_size=3)
		self.command = Twist()
		self.waypoint_viz_pub = rospy.Publisher('waypoint_marker', Marker, queue_size=10)

	def set_position_callback(self,marker,joy):
		print self.position_marker.ns
		print joy.buttons[3] == 1
		print marker.ns == "PEOPLE"
		if (self.position_marker.ns == "NONE" and joy.buttons[3] == 1 and marker.ns == "PEOPLE"):
			self.position_marker = marker
			print "set position"
			ref_marker = Marker()
			ref_marker.header.frame_id = "base_footprint"
			ref_marker.header.stamp = rospy.get_rostime()
			ref_marker.ns = "robot"
			ref_marker.type = 9
			ref_marker.action = 0
			ref_marker.pose.position.x = self.position_marker.pose.position.x
			ref_marker.pose.position.y = self.position_marker.pose.position.y
			ref_marker.pose.position.z = self.position_marker.pose.position.z
			ref_marker.scale.x = .25
			ref_marker.scale.y = .25
			ref_marker.scale.z = .25
			ref_marker.color.r = 1.0
			ref_marker.color.g = 0.0
			ref_marker.color.a = 1.0
			ref_marker.lifetime = rospy.Duration(0)
			self.ref_viz_pub.publish(ref_marker)
		else:
			pass

	def move_callback(self,marker):
		self.client.wait_for_server()
		# rospy.loginfo('got server')
		goal_pose = MoveBaseGoal()
		goal_pose.target_pose.header.stamp = rospy.Time.now()
		goal_pose.target_pose.header.frame_id = "base_footprint"
		goal_pose.target_pose.pose.position.x = 0
		goal_pose.target_pose.pose.position.y = 0
		goal_pose.target_pose.pose.position.z = 0

		goal_pose.target_pose.pose.orientation.x = 0
		goal_pose.target_pose.pose.orientation.y = 0
		goal_pose.target_pose.pose.orientation.z = 0
		goal_pose.target_pose.pose.orientation.w = 1

		waypoint_marker = Marker()
		waypoint_marker.header.frame_id = "base_footprint"
		waypoint_marker.header.stamp = rospy.get_rostime()
		waypoint_marker.ns = "robot"
		waypoint_marker.id = 1
		waypoint_marker.type = 9
		waypoint_marker.action = 0
		waypoint_marker.pose.position.x = 0
		waypoint_marker.pose.position.y = 0
		waypoint_marker.pose.position.z = 0
		waypoint_marker.pose.orientation.x = 0
		waypoint_marker.pose.orientation.y = 0
		waypoint_marker.pose.orientation.z = 0
		waypoint_marker.scale.x = .25
		waypoint_marker.scale.y = .25
		waypoint_marker.scale.z = .25
		waypoint_marker.color.r = 1.0
		waypoint_marker.color.g = 0.0
		waypoint_marker.color.b = 0.0
		waypoint_marker.color.a = 1.0
		waypoint_marker.lifetime = rospy.Duration(0)


		if (self.joy_bool and self.position_marker.ns != "NONE"):

			if (marker.pose.position.x >= (self.position_marker.pose.position.x+.1)):
				print "greater"
				# self.command.linear.x = .75
				goal_pose.target_pose.pose.position.x = 2
				waypoint_marker.pose.position.x = 2
				waypoint_marker.pose.orientation.w = 1

				self.client.send_goal(goal_pose)
				rospy.loginfo('sent forward goal')
				self.waypoint_viz_pub.publish(waypoint_marker)
				self.client.wait_for_result()
				# self.follower_twist.publish(self.command)
			elif (marker.pose.position.x <= (self.position_marker.pose.position.x-.1)):
				# self.command.linear.x = -.75
				print "Less Than"
				goal_pose.target_pose.pose.position.x = -2
				waypoint_marker.pose.position.x = -2
				waypoint_marker.pose.orientation.w = 1

				self.client.send_goal(goal_pose)
				rospy.loginfo('sent backwards goal')
				self.waypoint_viz_pub.publish(waypoint_marker)
				self.client.wait_for_result()
				# self.follower_twist.publish(self.command)
			else:
				self.client.cancel_goal()
				print "Canceled Goals"
				# self.command.linear.x = 0
				# self.follower_twist.publish(self.command)
				rospy.loginfo('sent stay goal')

			# print self.command.linear.x

		else:
			pass
			# print "Unactive Callback"
		# if (self.joy_bool == True):
		#     if (self.past_marker.ns == "NONE"):
		#         return 0
		#     else:
		#         x_vect = (marker.pose.position.x - self.past_marker.pose.position.x)
		#         # y_vect = (marker.pose.position.y - self.past_marker.pose.position.y)
		#         self.command.linear.x = x_vect
		#         # self.command.linear.y = y_vect
		#         self.follower_twist.publish(self.command)
		#
		# else:
		#     return 0

	def joy_callback(self,joy):
		if (joy.buttons[3] == 1):
			self.joy_bool = True
			# print self.joy_bool
		else:
			self.joy_bool = False
			if (self.position_marker.ns != "NONE"):
				# self.command.linear.x = 0
				# self.follower_twist.publish(self.command)
				# print "set vel to 0"
				self.client.cancel_goal()
				print "Canceled Goals"

			self.position_marker.ns = "NONE"
			# print self.joy_bool
	# def mem_callback(self,marker):
	#     if (marker.ns == "PEOPLE") == True:
	#         self.past_marker = marker
	#     else:
	#         return 0

if __name__ == '__main__':
	rospy.init_node('wheelchair_follower_controller',log_level=rospy.DEBUG)
	follower = follower()
	rospy.spin()
