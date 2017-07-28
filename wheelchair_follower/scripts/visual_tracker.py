#!/usr/bin/env python
import roslib
roslib.load_manifest('wheelchair_follower')
import rospy, message_filters
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Joy, Image
import numpy as np
import std_msgs.msg
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge, CvBridgeError
import cv2

class vfolllower:
    def __init__(self):
        self.bridge = CvBridge()
        self.camera = rospy.Subscriber('/img_throttle',Image,self.cam_callback)
    def cam_callback(self,stream):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(stream, "bgr8")
            height = np.size(cv_image, 0)
            width = np.size(cv_image, 1)
            # print height
            # print width

        except CvBridgeError as e:
            print(e)

        # CvPoint mid_bottom, mid_top
        mid_bottom_x = np.size(cv_image, 1)/2
        mid_bottom_y = 0
        mid_top_x = np.size(cv_image, 1)/2
        mid_top_y = np.size(cv_image, 0)
        cv2.line(cv_image, (mid_bottom_x,mid_bottom_y), (mid_top_x,mid_top_y), (0,0,255), 2);
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

if __name__ == '__main__':
	rospy.init_node('vfolllower',log_level=rospy.DEBUG)
	vfolllower = vfolllower()
	rospy.spin()
