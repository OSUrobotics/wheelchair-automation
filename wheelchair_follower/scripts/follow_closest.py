#!/usr/bin/python

import rospy as rp
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker
from tf import Transformer
from geometry_msgs.msg import Twist, Point, Quaternion
from sensor_msgs.msg import LaserScan
import math

class Follower:
    def __init__(self):
        self.laser_sub = rp.Subscriber("scan_multi", LaserScan, self.laser_callback)
        self.left_sub = rp.Subscriber("scan_filtered_left", LaserScan, self.left_callback)
        self.right_sub = rp.Subscriber("scan_filtered_right", LaserScan, self.right_callback)
        self.vel_pub =  rp.Publisher("input/misc/cmd_vel", Twist, queue_size = 1)
        self.mark_pub = rp.Publisher("marker/arrow", Marker, queue_size = 1)
        self.left_scan = []
        self.right_scan = []
        self.index = 0
        self.min_scan = 0.0
        self.index_location = ""
        self.left_angle_increment = 0.0
        self.right_angle_increment = 0.0
        self.left_min_angle = 0.0
        self.right_min_angle = 0.0
        self.angular_gain = 0.8
        self.linear_gain = 1.3
        self.counter = 0
        self.command = Twist()
        self.arrow = Marker()

        self.arrow.header.frame_id = 'base_footprint'
        self.arrow.ns = 'wheelchair'

        self.arrow.type = 0
        self.arrow.id = 300
        self.arrow.scale.z = 0.1
        self.arrow.scale.y = 0.1
        self.arrow.scale.x = 1.0
        self.arrow.action = 0
        self.arrow.pose.position.x = 0.0
        self.arrow.pose.position.y = 0.0
        self.arrow.pose.position.z = 0.0
        self.arrow.lifetime = rp.Duration(0)
        self.arrow.color.r = 99.0
        self.arrow.color.g = 152.0
        self.arrow.color.b = 255.0
        self.arrow.color.a = 1.0

    def laser_callback(self, scan):
        try:
            self.min_scan = min(scan.ranges)
        except:
            self.command.angular.z = 1.0
            self.command.linear.x = 0.0
            self.goto()
            return
        if self.min_scan > 10.0:
            self.command.angular.z = 1.0
            self.command.linear.x = 0.0
            self.goto()
            return
        try:
            self.index = self.left_scan.index(self.min_scan)
            self.index_location = "left"
        except:
            try:
                self.index = self.right_scan.index(self.min_scan)
                self.index_location = "right"
            except:
                try:
                    self.min_scan = min(self.left_scan)
                    self.index = self.left_scan.index(self.min_scan)
                    self.index_location = "left"
                    try:
                        if self.min_scan < min(self.right_scan):
                            self.min_scan = min(self.right_scan)
                            self.index = self.right_scan.index(self.min_scan)
                            self.index_location = "right"
                    except:
                        pass
                except:
                    pass

        self.gen_command()
        self.goto()

    def left_callback(self, left_scan_msg):
        self.left_scan = list(left_scan_msg.ranges)
        for i in range(len(self.left_scan)):
            if str(self.left_scan[i]) == 'nan':
                self.left_scan[i] = 125.0
        self.left_angle_increment = left_scan_msg.angle_increment
        self.left_min_angle = left_scan_msg.angle_min

    def right_callback(self, right_scan_msg):
        self.right_scan = list(right_scan_msg.ranges)
        for i in range(len(self.right_scan)):
            if str(self.right_scan[i]) == 'nan':
                self.right_scan[i] = 250.0
        self.right_angle_increment = right_scan_msg.angle_increment
        self.right_min_angle = right_scan_msg.angle_min

    def gen_command(self):
        if self.index_location == "none":
            if self.counter > 10:
                self.counter = 0
                self.command.linear.x = 0.0
                self.command.angular.x = 0.0
                return
            else:
                self.counter += 1
        elif self.index_location == "left":
            goto_angle = (self.left_min_angle - math.pi/2) + self.index * self.left_angle_increment
            self.command.angular.z = goto_angle * self.angular_gain
        elif self.index_location =="right":
            goto_angle = (self.right_min_angle + math.pi/2) + self.index * self.right_angle_increment
            self.command.angular.z = goto_angle * self.angular_gain
        else:
            goto_angle = 0.0
            self.command.angular.z = 0.0

        if self.command.angular.z > 1.5:
            self.command.angular.z = 1.5
        elif self.command.angular.z < -1.5:
            self.command.angular.z = -1.5

        if self.min_scan > 0.85 and abs(goto_angle) < 0.3 and self.min_scan <= 10:
            self.command.linear.x = (self.min_scan - 0.85) * self.linear_gain
            if self.command.linear.x < 0.2 and self.command.linear.x > 0.0:
                self.command.linear.x = 0.2
            elif self.command.linear.x > 1.5:
                self.command.linear.x = 1.5
        else:
            self.command.linear.x = 0.0

        if self.min_scan < 0.95 and abs(goto_angle) < 0.3:
            self.command.linear.x = 0.0
            self.command.angular.z = 0.0

        if self.min_scan == 125:
            self.command.angular.z = 1.0
            self.command.linear.x = 0.0
        elif self.min_scan == 250:
            self.command.angular.z = -1.0
            self.command.linear.x = 0.0

        quat = quaternion_from_euler(0.0, 0.0, goto_angle)

        self.arrow.pose.orientation.x = float(quat[0])
        self.arrow.pose.orientation.y = float(quat[1])
        self.arrow.pose.orientation.z = float(quat[2])
        self.arrow.pose.orientation.w = float(quat[3])
        self.arrow.scale.x = float(self.min_scan)

    def goto(self):
        self.arrow.action = Marker.DELETE
        self.mark_pub.publish(self.arrow)
        self.vel_pub.publish(self.command)
        self.arrow.action = Marker.ADD
        self.mark_pub.publish(self.arrow)
        print "Min Scan: " + str(self.min_scan) + "\nGoto angle: " + str(self.command.angular.z/self.angular_gain)



if __name__ == '__main__':
    rp.init_node('wheelchair_follower', log_level=rp.DEBUG)
    follower = Follower()
    rp.spin()
